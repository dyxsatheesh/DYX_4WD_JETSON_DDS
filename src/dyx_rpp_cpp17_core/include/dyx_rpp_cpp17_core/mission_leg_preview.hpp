#pragma once
// Phase 2F3A: C++17 deterministic, non-actuating single-leg mission preview.
// Integrates already-tested geometry, guidance, tracking, speed, terminal stop,
// and forward-only yaw-rate. No ROS, motor writer, spray, or mission transition.
// The supplied ENU position MUST be the calibrated rear GNSS MARKING reference,
// not an uncorrected PX4 IMU, chassis centre, or front GNSS antenna position.
// All positive outputs are DIAGNOSTIC PREVIEWS; not authorized motor commands.

#include "dyx_rpp_cpp17_core/forward_yaw_rate.hpp"
#include "dyx_rpp_cpp17_core/path_index.hpp"
#include "dyx_rpp_cpp17_core/precision_guidance.hpp"
#include "dyx_rpp_cpp17_core/speed_regulator.hpp"
#include "dyx_rpp_cpp17_core/terminal_stop.hpp"
#include "dyx_rpp_cpp17_core/tracking_stability.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dyx::rpp {

struct MissionLegPreviewConfig {
  double cruise_speed_mps{0.60};
  double max_telemetry_gap_sec{0.25};
  double terminal_engage_distance_m{0.75};
  PrecisionGuidanceConfig guidance{};
  LongitudinalRegulatorConfig speed{};
  TrackingControlConfig tracking{};
  RadialStopConfig terminal{};
  ForwardYawConfig yaw{};
  void validate() const {
    require_finite(cruise_speed_mps, "cruise_speed_mps");
    require_finite(max_telemetry_gap_sec, "max_telemetry_gap_sec");
    require_finite(terminal_engage_distance_m, "terminal_engage_distance_m");
    if (!(cruise_speed_mps > 0.0 && cruise_speed_mps <= 1.0) ||
        max_telemetry_gap_sec <= 0.0 || max_telemetry_gap_sec > 0.5 ||
        terminal_engage_distance_m < terminal.radial_tolerance_m ||
        terminal_engage_distance_m > terminal.terminal_guidance_distance_m)
      throw std::invalid_argument("unsafe mission-leg preview configuration");
    speed.validate(); tracking.validate(); terminal.validate(); yaw.validate();
  }
};

struct MissionLegPreviewInput {
  double now_sec{};
  double dt_sec{};
  double position_sample_time_sec{};
  Point2D marking_reference_enu{};  // rear GNSS marking reference
  double rover_yaw_enu_rad{};
  double measured_forward_speed_mps{};
  double position_derived_speed_mps{};
  double measured_yaw_rate_enu_radps{};
  bool marking_reference_calibrated{false};
  bool telemetry_fresh{false};
  bool rtk_fixed{false};
  bool mission_enabled{false};
  bool battery_ok{false};
  bool obstacle_clear{false};
  bool emergency_stop{true};
  bool stationary_pivot_permitted{false};
};

struct MissionLegPreviewResult {
  bool valid{false};
  bool hold_zero{true};
  bool terminal_certified{false};
  bool pivot_complete{false};
  std::string reason{"UNINITIALIZED"};
  std::string goal_identity{};
  double radial_error_m{};
  double along_remaining_m{};
  double signed_cross_track_m{};
  double path_progress_m{};
  double forward_preview_mps{};
  double velocity_north_preview_mps{};
  double velocity_east_preview_mps{};
  double yaw_rate_enu_preview_radps{};
  std::string speed_cap_owner{};
};

class MissionLegPreview {
 public:
  // Explicit raw-index goal: must be an identifiable marking anchor.
  MissionLegPreview(std::vector<RawPathPoint> path, int goal_raw_index,
                    MissionLegPreviewConfig cfg = {})
      : config_(std::move(cfg)),
        geometry_(PathGeometryIndex::build(path)),
        progress_(geometry_),
        span_(geometry_.active_span(goal_raw_index)),
        speed_(config_.speed), tracking_(config_.tracking),
        terminal_(config_.terminal), yaw_(config_.yaw) {
    config_.validate();
    const auto *anchor = geometry_.semantic_anchor_at(goal_raw_index);
    if (!anchor || anchor->point_type != point_marking || anchor->identity.empty() ||
        !span_.last_segment_index || span_.active_goal_identity.empty())
      throw std::invalid_argument("mission leg must end at a reachable marking anchor");
    goal_ = anchor->point;
    const auto &last_seg = geometry_.segments.at(
        static_cast<std::size_t>(*span_.last_segment_index));
    final_bearing_ = last_seg.heading_rad;
    // The lateral error for certification is evaluated relative to the
    // final inbound segment, not a virtual lookahead/extended endpoint.
    reset_diagnostics();
  }

  MissionLegPreview(const MissionLegPreview &) = delete;
  MissionLegPreview &operator=(const MissionLegPreview &) = delete;
  MissionLegPreview(MissionLegPreview &&) = delete;
  MissionLegPreview &operator=(MissionLegPreview &&) = delete;

  MissionLegPreviewResult step(const MissionLegPreviewInput &s) {
    if (latched_) return hold("SAFETY_LATCHED");
    if (!finite_input(s)) return latch("INVALID_INPUT");
    if (!s.marking_reference_calibrated) return latch("MARKING_REFERENCE_UNCALIBRATED");
    if (!s.telemetry_fresh || !s.rtk_fixed || !s.mission_enabled ||
        !s.battery_ok || !s.obstacle_clear || s.emergency_stop)
      return latch("MISSION_SAFETY_GATE");
    if (s.now_sec < 0.0 || s.dt_sec < 0.0 ||
        s.position_derived_speed_mps < 0.0 || s.measured_forward_speed_mps < 0.0 ||
        s.position_sample_time_sec < 0.0 ||
        (last_time_ && (s.now_sec < *last_time_ ||
          s.now_sec - *last_time_ > config_.max_telemetry_gap_sec)))
      return latch("INVALID_TIME_OR_SPEED");
    if (last_position_sample_ && s.position_sample_time_sec < *last_position_sample_)
      return latch("POSITION_TIME_REVERSED");
    // Never allow an incremented position sample to appear from the future.
    if (s.position_sample_time_sec > s.now_sec + 1e-6)
      return latch("POSITION_TIME_IN_FUTURE");
    if (s.now_sec - s.position_sample_time_sec > config_.max_telemetry_gap_sec)
      return latch("POSITION_STALE");
    const double actual_dt = last_time_ ? s.now_sec - *last_time_ : 0.0;
    last_time_ = s.now_sec;
    last_position_sample_ = s.position_sample_time_sec;

    try {
      const auto projection = progress_.update(s.marking_reference_enu, span_);
      const auto guidance = compute_precision_guidance(
          config_.guidance, geometry_, projection, span_,
          s.marking_reference_enu, s.rover_yaw_enu_rad,
          std::abs(s.measured_forward_speed_mps));
      const double dx = goal_.x - s.marking_reference_enu.x;
      const double dy = goal_.y - s.marking_reference_enu.y;
      const double along = dx * std::cos(final_bearing_) + dy * std::sin(final_bearing_);
      const double cross = dx * -std::sin(final_bearing_) + dy * std::cos(final_bearing_);
      const double radial = std::hypot(dx, dy);
      terminal_active_ = terminal_active_ ||
          (projection.remaining_to_active_stop_m <= config_.terminal_engage_distance_m);

      const auto tracking = tracking_.step({span_.active_goal_identity,
          projection.progress_s, projection.signed_cross_track_m,
          guidance.heading_error_rad, last_forward_preview_mps_,
          s.measured_forward_speed_mps, guidance.bearing_clamp_fired,
          true, std::min(s.dt_sec, actual_dt), false});
      if (!tracking.valid) return latch("TRACKING_INVALID");

      SpeedRegulatorInput si{};
      si.mission_speed_ceiling_mps = config_.cruise_speed_mps;
      si.measured_speed_mps = s.measured_forward_speed_mps;
      si.last_commanded_speed_mps = last_forward_preview_mps_;
      si.dt_sec = std::min(s.dt_sec, actual_dt);
      si.along_track_progress_m = projection.progress_s;
      si.heading_error_rad = guidance.heading_error_rad;
      si.cross_track_error_m = projection.signed_cross_track_m;
      si.distance_to_terminal_m = projection.remaining_to_active_stop_m;
      si.distance_to_corner_m = projection.next_corner_distance_m;
      si.corner_angle_rad = projection.next_corner_angle_rad;
      si.tracking_acceleration_allowed = tracking.acceleration_allowed;
      si.tracking_speed_cap_mps = config_.cruise_speed_mps * tracking.recovery_speed_scale;
      const auto speed = speed_.resolve(si);

      const auto radial_stop = terminal_.step({s.now_sec,
          s.position_sample_time_sec, terminal_active_, span_.active_goal_identity,
          along, cross, s.marking_reference_enu.x, s.marking_reference_enu.y,
          s.position_derived_speed_mps, s.measured_yaw_rate_enu_radps,
          speed.requested_speed_mps, true});
      if (radial_stop.failure != RadialStopFailure::none)
        return latch("TERMINAL_STOP_FAILED");
      const double terminal_speed = radial_stop.forward_speed_command_mps;
      if (!std::isfinite(terminal_speed) || terminal_speed < 0.0)
        return latch("NONFORWARD_TERMINAL_COMMAND");
      const bool stopped = radial_stop.hold_zero;
      // Only a future mission manager may authorize the next leg and pivot.
      // This preview never changes the active marking goal automatically.
      const double target_yaw = s.stationary_pivot_permitted ? final_bearing_ :
          guidance.limited_command_bearing_rad;
      const auto yaw = yaw_.step({s.now_sec, std::min(s.dt_sec, actual_dt),
          s.rover_yaw_enu_rad, target_yaw,
          s.measured_yaw_rate_enu_radps, terminal_speed,
          true, true, s.stationary_pivot_permitted, stopped});
      if (yaw.safe_hold) return latch("YAW_SAFETY_LATCHED");
      if (stopped && (yaw.forward_speed_mps != 0.0 || yaw.yaw_rate_enu_radps != 0.0))
        return latch("TERMINAL_STOP_OVERRIDDEN");
      last_forward_preview_mps_ = yaw.forward_speed_mps;
      return {true, stopped, radial_stop.certificate.has_value(),
          yaw.pivot_complete, radial_stop.certificate ? "RADIAL_STOP_CERTIFIED" : yaw.reason,
          span_.active_goal_identity, radial, along, projection.signed_cross_track_m,
          projection.progress_s, yaw.forward_speed_mps,
          yaw.velocity_north_mps, yaw.velocity_east_mps,
          yaw.yaw_rate_enu_radps, speed.winning_cap_owner};
    } catch (const std::exception &) {
      return latch("INTEGRATION_EXCEPTION");
    }
  }

  bool latched() const { return latched_; }
  const std::string &goal_identity() const { return span_.active_goal_identity; }
  // No automatic reset/retry on safety faults. Create a new object only when
  // a higher-level, externally verified mission-control boundary permits it.

 private:
  static bool finite_input(const MissionLegPreviewInput &s) {
    for (double v : {s.now_sec, s.dt_sec, s.position_sample_time_sec,
         s.marking_reference_enu.x, s.marking_reference_enu.y,
         s.rover_yaw_enu_rad, s.measured_forward_speed_mps,
         s.position_derived_speed_mps, s.measured_yaw_rate_enu_radps}) {
      if (!std::isfinite(v)) return false;
    }
    return true;
  }
  void reset_diagnostics() {
    progress_.reset(GeometryResetReason::initial_install);
    speed_.reset(span_.start_s, 0.0);
    tracking_.reset(span_.active_goal_identity);
    terminal_.reset(); yaw_.reset();
    terminal_active_ = false;
    last_forward_preview_mps_ = 0.0;
    last_time_.reset(); last_position_sample_.reset(); latched_ = false;
  }
  MissionLegPreviewResult hold(const char *reason) const {
    MissionLegPreviewResult out{};
    out.reason = reason;
    out.goal_identity = span_.active_goal_identity;
    return out;
  }
  MissionLegPreviewResult latch(const char *reason) {
    latched_ = true;
    last_forward_preview_mps_ = 0.0;
    return hold(reason);
  }

  MissionLegPreviewConfig config_{};
  PathGeometryIndex geometry_;
  GeometryProgressTracker progress_;
  ActiveSemanticSpan span_{};
  Point2D goal_{};
  double final_bearing_{};
  LongitudinalRegulator speed_;
  TrackingStabilityController tracking_;
  TerminalStopRegulator terminal_;
  ForwardYawRatePlanner yaw_;
  bool terminal_active_{false};
  bool latched_{false};
  double last_forward_preview_mps_{};
  std::optional<double> last_time_{};
  std::optional<double> last_position_sample_{};
};

}  // namespace dyx::rpp
