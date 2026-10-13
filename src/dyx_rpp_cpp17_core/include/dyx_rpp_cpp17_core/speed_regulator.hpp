#pragma once
// Phase 2C2: non-actuating C++17 port of Python speed_regulator.py.
// Keep implementation and all thresholds synchronized with the Python source.
#include "dyx_rpp_cpp17_core/primitives.hpp"
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dyx::rpp {

struct LongitudinalRegulatorConfig {
  double hardware_speed_ceiling_mps{1.0};
  double acceleration_mps2{0.75};
  double deceleration_mps2{0.75};
  double launch_speed_mps{0.10};
  double control_dt_max_sec{0.10};
  double heading_accel_full_error_rad{2.0 * pi / 180.0};
  double heading_recovery_start_rad{4.0 * pi / 180.0};
  double heading_recovery_full_rad{15.0 * pi / 180.0};
  double cross_track_accel_full_m{0.010};
  double cross_track_recovery_start_m{0.020};
  double cross_track_recovery_full_m{0.100};
  double recovery_min_speed_mps{0.15};
  double recovery_exit_dwell_sec{0.30};
  double corner_angle_threshold_rad{45.0 * pi / 180.0};
  double corner_target_speed_mps{0.12};
  double corner_accel_block_buffer_m{0.10};
  double terminal_target_speed_mps{0.0};
  double braking_latency_sec{0.10};
  double braking_margin_m{0.05};
  bool curvature_enabled{false};
  double lateral_acceleration_max_mps2{0.30};
  double curvature_epsilon_inv_m{1.0e-6};

  void validate() const {
    for (double v : {hardware_speed_ceiling_mps, acceleration_mps2,
      deceleration_mps2, launch_speed_mps, control_dt_max_sec,
      heading_accel_full_error_rad, heading_recovery_start_rad,
      heading_recovery_full_rad, cross_track_accel_full_m,
      cross_track_recovery_start_m, cross_track_recovery_full_m,
      recovery_min_speed_mps, recovery_exit_dwell_sec,
      corner_angle_threshold_rad, corner_target_speed_mps,
      corner_accel_block_buffer_m, terminal_target_speed_mps,
      braking_latency_sec, braking_margin_m,
      lateral_acceleration_max_mps2, curvature_epsilon_inv_m}) {
      require_finite(v, "speed regulator config must be finite");
    }
    for (double v : {hardware_speed_ceiling_mps, acceleration_mps2,
      deceleration_mps2, control_dt_max_sec, heading_recovery_full_rad,
      cross_track_recovery_full_m, recovery_exit_dwell_sec,
      lateral_acceleration_max_mps2, curvature_epsilon_inv_m}) {
      if (!(v > 0.0)) throw std::invalid_argument("invalid positive speed configuration");
    }
    for (double v : {launch_speed_mps, heading_accel_full_error_rad,
      heading_recovery_start_rad, cross_track_accel_full_m,
      cross_track_recovery_start_m, recovery_min_speed_mps,
      corner_angle_threshold_rad, corner_target_speed_mps,
      corner_accel_block_buffer_m, terminal_target_speed_mps,
      braking_latency_sec, braking_margin_m}) {
      if (v < 0.0) throw std::invalid_argument("negative speed configuration");
    }
    if (!(heading_accel_full_error_rad <= heading_recovery_start_rad &&
          heading_recovery_start_rad < heading_recovery_full_rad &&
          heading_recovery_full_rad <= pi &&
          cross_track_accel_full_m <= cross_track_recovery_start_m &&
          cross_track_recovery_start_m < cross_track_recovery_full_m &&
          corner_angle_threshold_rad <= pi)) {
      throw std::invalid_argument("speed configuration thresholds out of order");
    }
    for (double v : {launch_speed_mps, recovery_min_speed_mps,
                     corner_target_speed_mps, terminal_target_speed_mps}) {
      if (v > hardware_speed_ceiling_mps) {
        throw std::invalid_argument("configured speed exceeds hardware ceiling");
      }
    }
  }
};

struct SpeedRegulatorInput {
  double mission_speed_ceiling_mps{};
  double measured_speed_mps{};
  double last_commanded_speed_mps{};
  double dt_sec{};
  double along_track_progress_m{};
  double heading_error_rad{};
  double cross_track_error_m{};
  std::optional<double> distance_to_corner_m{};
  std::optional<double> corner_angle_rad{};
  std::optional<double> distance_to_terminal_m{};
  std::optional<double> terminal_target_speed_override_mps{};
  std::optional<double> curvature_inv_m{};
  bool tracking_acceleration_allowed{true};
  std::optional<double> tracking_speed_cap_mps{};
  bool hard_zero{false};
};

struct SpeedCaps {
  double mission_mps{}, hardware_mps{}, acceleration_mps{};
  double heading_mps{}, cross_track_mps{};
  std::optional<double> recovery_mps{}, tracking_mps{}, corner_mps{},
    terminal_mps{}, curvature_mps{};
};

struct SpeedRegulatorResult {
  double requested_speed_mps{};
  std::string winning_cap_owner{};
  SpeedCaps caps{};
  double effective_speed_mps{};
  double bounded_dt_sec{};
  double acceleration_gate_scale{};
  double acceleration_progress_m{};
  bool recovery_active{};
  std::string recovery_transition{};
  double recovery_exit_dwell_sec{};
  double recovery_exit_dwell_required_sec{};
  std::optional<double> corner_required_braking_distance_m{};
  std::optional<double> terminal_required_braking_distance_m{};
};

class LongitudinalRegulator {
 public:
  explicit LongitudinalRegulator(LongitudinalRegulatorConfig config = {})
    : config_(std::move(config)) { config_.validate(); }

  void reset(double along_track_progress_m = 0.0, double initial_speed_mps = 0.0) {
    require_nonnegative(along_track_progress_m, "along_track_progress_m");
    require_nonnegative(initial_speed_mps, "initial_speed_mps");
    if (initial_speed_mps > config_.hardware_speed_ceiling_mps)
      throw std::invalid_argument("initial speed exceeds hardware ceiling");
    start_progress_m_ = along_track_progress_m;
    initial_speed_mps_ = initial_speed_mps;
    last_requested_speed_mps_ = initial_speed_mps;
    recovery_active_ = false;
    recovery_exit_dwell_sec_ = 0.0;
  }

  SpeedRegulatorResult resolve(const SpeedRegulatorInput &r) {
    const auto &c = config_;
    require_nonnegative(r.mission_speed_ceiling_mps, "mission_speed_ceiling_mps");
    require_finite(r.measured_speed_mps, "measured_speed_mps");
    require_finite(r.last_commanded_speed_mps, "last_commanded_speed_mps");
    require_finite(r.dt_sec, "dt_sec");
    require_nonnegative(r.along_track_progress_m, "along_track_progress_m");
    require_finite(r.heading_error_rad, "heading_error_rad");
    require_finite(r.cross_track_error_m, "cross_track_error_m");
    for (const auto &value : {r.distance_to_corner_m, r.distance_to_terminal_m,
         r.terminal_target_speed_override_mps, r.tracking_speed_cap_mps}) {
      if (value) require_nonnegative(*value, "optional distance or speed");
    }
    for (const auto &value : {r.corner_angle_rad, r.curvature_inv_m}) {
      if (value) require_finite(*value, "optional angle or curvature");
    }
    const double terminal_target =
      r.terminal_target_speed_override_mps.value_or(c.terminal_target_speed_mps);
    if (terminal_target > c.hardware_speed_ceiling_mps)
      throw std::invalid_argument("terminal override exceeds hardware ceiling");

    const double mission = r.mission_speed_ceiling_mps;
    const double measured = std::abs(r.measured_speed_mps);
    const double last = std::abs(r.last_commanded_speed_mps);
    const double dt = std::min(std::max(0.0, r.dt_sec), c.control_dt_max_sec);
    const double base = std::min(mission, c.hardware_speed_ceiling_mps);
    const double effective = std::max(measured, last);
    const double progress = std::max(0.0, r.along_track_progress_m - start_progress_m_);
    const double heading = std::abs(r.heading_error_rad);
    const double cross = std::abs(r.cross_track_error_m);
    const double heading_gate = accel_gate(heading, c.heading_accel_full_error_rad,
                                             c.heading_recovery_full_rad);
    const double cross_gate = accel_gate(cross, c.cross_track_accel_full_m,
                                           c.cross_track_recovery_full_m);

    const bool was_recovering = recovery_active_;
    double dwell_result = recovery_exit_dwell_sec_;
    const bool enter = heading >= c.heading_recovery_full_rad ||
                       cross >= c.cross_track_recovery_full_m;
    const bool exit_geometry = heading <= c.heading_recovery_start_rad &&
                               cross <= c.cross_track_recovery_start_m;
    if (recovery_active_) {
      if (exit_geometry) {
        recovery_exit_dwell_sec_ = std::min(c.recovery_exit_dwell_sec,
                                            recovery_exit_dwell_sec_ + dt);
      } else recovery_exit_dwell_sec_ = 0.0;
      dwell_result = recovery_exit_dwell_sec_;
      if (recovery_exit_dwell_sec_ >= c.recovery_exit_dwell_sec) {
        recovery_active_ = false;
        recovery_exit_dwell_sec_ = 0.0;
      }
    } else if (enter) {
      recovery_active_ = true;
      recovery_exit_dwell_sec_ = 0.0;
      dwell_result = 0.0;
    }
    const std::string transition = recovery_active_ ?
      (was_recovering ? "ACTIVE" : "ENTERED") :
      (was_recovering ? "EXITED" : "INACTIVE");

    std::optional<double> corner_cap, corner_required;
    double corner_gate = 1.0;
    if (r.distance_to_corner_m && r.corner_angle_rad &&
        std::abs(*r.corner_angle_rad) >= c.corner_angle_threshold_rad) {
      corner_required = braking_distance(effective, c.corner_target_speed_mps,
                         c.deceleration_mps2, c.braking_latency_sec, c.braking_margin_m);
      corner_cap = std::min(base, allowable_speed_for_distance(
                      *r.distance_to_corner_m, c.corner_target_speed_mps,
                      c.deceleration_mps2, c.braking_latency_sec, c.braking_margin_m));
      if (*r.distance_to_corner_m <= *corner_required + c.corner_accel_block_buffer_m)
        corner_gate = 0.0;
    }
    std::optional<double> terminal_cap, terminal_required;
    double terminal_gate = 1.0;
    if (r.distance_to_terminal_m) {
      terminal_required = braking_distance(effective, terminal_target,
                          c.deceleration_mps2, c.braking_latency_sec, c.braking_margin_m);
      terminal_cap = std::min(base, allowable_speed_for_distance(
                       *r.distance_to_terminal_m, terminal_target,
                       c.deceleration_mps2, c.braking_latency_sec, c.braking_margin_m));
      if (*r.distance_to_terminal_m <= *terminal_required) terminal_gate = 0.0;
    }
    const double gate = std::min({heading_gate, cross_gate, corner_gate, terminal_gate});
    const double time_accel_cap = last_requested_speed_mps_ + gate * c.acceleration_mps2 * dt;
    const double dist_accel_cap = std::sqrt(
        std::pow(std::max(c.launch_speed_mps, initial_speed_mps_), 2) +
        2.0 * c.acceleration_mps2 * progress);
    double accel_cap = std::min({base, time_accel_cap, dist_accel_cap});
    if (!r.tracking_acceleration_allowed) accel_cap = std::min(accel_cap, last);
    const double heading_cap = decreasing_cap(heading,
      c.heading_recovery_start_rad, c.heading_recovery_full_rad, base, c.recovery_min_speed_mps);
    const double cross_cap = decreasing_cap(cross,
      c.cross_track_recovery_start_m, c.cross_track_recovery_full_m, base, c.recovery_min_speed_mps);
    std::optional<double> curvature_cap;
    if (c.curvature_enabled && r.curvature_inv_m &&
        std::abs(*r.curvature_inv_m) >= c.curvature_epsilon_inv_m) {
      curvature_cap = std::min(base, std::sqrt(c.lateral_acceleration_max_mps2 /
                                                  std::abs(*r.curvature_inv_m)));
    }

    SpeedCaps caps;
    caps.mission_mps = mission;
    caps.hardware_mps = c.hardware_speed_ceiling_mps;
    caps.recovery_mps = recovery_active_ ?
      std::optional<double>(std::min(base, c.recovery_min_speed_mps)) : std::nullopt;
    caps.acceleration_mps = accel_cap;
    caps.heading_mps = heading_cap;
    caps.cross_track_mps = cross_cap;
    caps.tracking_mps = r.tracking_speed_cap_mps ?
      std::optional<double>(std::min(base, *r.tracking_speed_cap_mps)) : std::nullopt;
    caps.corner_mps = corner_cap;
    caps.terminal_mps = terminal_cap;
    caps.curvature_mps = curvature_cap;

    // Strict < comparison preserves Python's first-candidate precedence on ties.
    std::pair<std::string, double> winner{"mission", caps.mission_mps};
    const auto candidate = [&](const char *name, double speed) {
      if (speed < winner.second) winner = {name, speed};
    };
    if (r.hard_zero) winner = {"hard_zero", 0.0};
    else if (recovery_active_) {
      candidate("hardware", caps.hardware_mps);
      if (caps.terminal_mps) candidate("terminal_braking", *caps.terminal_mps);
      if (caps.corner_mps) candidate("corner_preview", *caps.corner_mps);
      if (caps.recovery_mps) candidate("recovery", *caps.recovery_mps);
      if (caps.tracking_mps) candidate("tracking_stability", *caps.tracking_mps);
      if (caps.curvature_mps) candidate("curvature", *caps.curvature_mps);
    } else {
      candidate("hardware", caps.hardware_mps);
      candidate("acceleration", caps.acceleration_mps);
      candidate("heading_alignment", caps.heading_mps);
      candidate("cross_track_recovery", caps.cross_track_mps);
      if (caps.recovery_mps) candidate("recovery", *caps.recovery_mps);
      if (caps.tracking_mps) candidate("tracking_stability", *caps.tracking_mps);
      if (caps.corner_mps) candidate("corner_preview", *caps.corner_mps);
      if (caps.terminal_mps) candidate("terminal_braking", *caps.terminal_mps);
      if (caps.curvature_mps) candidate("curvature", *caps.curvature_mps);
    }
    const double speed = std::min(winner.second, base);
    if (!std::isfinite(speed) || speed < 0.0)
      throw std::runtime_error("invalid resolved speed");
    last_requested_speed_mps_ = speed;
    return {speed, winner.first, caps, effective, dt, gate, progress,
      recovery_active_, transition, dwell_result, c.recovery_exit_dwell_sec,
      corner_required, terminal_required};
  }

 private:
  static double accel_gate(double mag, double full, double blocked) {
    if (mag <= full) return 1.0;
    if (mag >= blocked) return 0.0;
    return 1.0 - (mag - full) / (blocked - full);
  }
  static double decreasing_cap(double mag, double start, double full,
                                double ceiling, double floor) {
    if (mag <= start) return ceiling;
    if (mag >= full) return std::min(ceiling, floor);
    const double ratio = (mag - start) / (full - start);
    return std::min(ceiling, ceiling + ratio * (floor - ceiling));
  }
  LongitudinalRegulatorConfig config_;
  double start_progress_m_{0.0};
  double initial_speed_mps_{0.0};
  double last_requested_speed_mps_{0.0};
  bool recovery_active_{false};
  double recovery_exit_dwell_sec_{0.0};
};
} // namespace dyx::rpp
