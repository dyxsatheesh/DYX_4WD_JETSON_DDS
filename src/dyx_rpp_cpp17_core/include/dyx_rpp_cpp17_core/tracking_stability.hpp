#pragma once

// Phase 2C3: a non-actuating, stateful C++17 port of
// rpp_controller/tracking_control.py::TrackingStabilityController.
// The run-level TrackingMetricsAccumulator is deliberately not ported here.
// This header must not independently command a PX4 rover or certify mission stops.

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace dyx::rpp {

constexpr double tracking_pi = 3.141592653589793238462643383279502884;

enum class TrackingControlState { tracking, recovery, recapture_stable };

inline const char *tracking_state_name(TrackingControlState state) {
  switch (state) {
    case TrackingControlState::tracking: return "tracking";
    case TrackingControlState::recovery: return "recovery";
    case TrackingControlState::recapture_stable: return "recapture_stable";
  }
  return "unknown";
}

struct TrackingControlConfig {
  double recovery_enter_cross_track_m{0.050};
  double recovery_exit_cross_track_m{0.020};
  double recovery_enter_heading_error_rad{tracking_pi / 12.0};  // 15 degrees
  double recovery_exit_heading_error_rad{tracking_pi / 36.0}; // 5 degrees
  bool recovery_enter_on_bearing_clamp{true};
  double stable_recapture_dwell_sec{0.30};
  double control_dt_max_sec{0.10};
  double recovery_speed_scale{0.35};
  double recapture_speed_scale{0.50};

  // Configuration fields for diagnostics are intentionally deferred with the
  // TrackingMetricsAccumulator port; no metrics are computed by this class.
  void validate() const {
    for (double v : {recovery_enter_cross_track_m,
                     recovery_exit_cross_track_m,
                     recovery_enter_heading_error_rad,
                     recovery_exit_heading_error_rad,
                     stable_recapture_dwell_sec,
                     control_dt_max_sec,
                     recovery_speed_scale,
                     recapture_speed_scale}) {
      if (!std::isfinite(v)) throw std::invalid_argument("nonfinite tracking configuration");
    }
    if (!(0.0 <= recovery_exit_cross_track_m &&
          recovery_exit_cross_track_m < recovery_enter_cross_track_m &&
          0.0 <= recovery_exit_heading_error_rad &&
          recovery_exit_heading_error_rad < recovery_enter_heading_error_rad &&
          recovery_enter_heading_error_rad <= tracking_pi)) {
      throw std::invalid_argument("tracking entry/exit thresholds out of order");
    }
    if (stable_recapture_dwell_sec < 0.0 || control_dt_max_sec <= 0.0 ||
        recovery_speed_scale < 0.0 || recovery_speed_scale > 1.0 ||
        recapture_speed_scale < 0.0 || recapture_speed_scale > 1.0 ||
        recovery_speed_scale > recapture_speed_scale) {
      throw std::invalid_argument("tracking dwell, dt or scale out of range");
    }
  }
};

struct TrackingControlInput {
  std::string path_identity{};
  double projection_s_m{};
  double signed_cross_track_m{};
  double heading_error_rad{};
  double commanded_speed_mps{};
  double measured_speed_mps{};
  bool bearing_clamped{false};
  bool telemetry_fresh{false};
  double dt_sec{};
  bool reset{false};
};

struct TrackingControlOutput {
  TrackingControlState state{TrackingControlState::recapture_stable};
  bool valid{false};
  bool acceleration_allowed{false};
  double recovery_speed_scale{0.0};
  std::string transition_reason{};
  double bounded_dt_sec{};
  double stable_dwell_sec{};
  bool reset_applied{false};
  std::optional<std::string> invalid_reason{};
};

class TrackingStabilityController {
 public:
  explicit TrackingStabilityController(TrackingControlConfig config = {})
      : config_(std::move(config)) { config_.validate(); }

  TrackingControlState state() const { return state_; }

  void reset(std::optional<std::string> path_identity = {}) {
    state_ = TrackingControlState::recapture_stable;
    stable_dwell_sec_ = 0.0;
    path_identity_ = std::move(path_identity);
  }

  TrackingControlOutput step(const TrackingControlInput &s) {
    const bool reset_applied = s.reset || path_identity_ != std::optional<std::string>(s.path_identity);
    if (reset_applied) reset(s.path_identity);

    const auto invalid = invalid_reason(s);
    if (invalid) {
      state_ = TrackingControlState::recovery;
      stable_dwell_sec_ = 0.0;
      return {state_, false, false, 0.0, "invalid_input", 0.0,
              0.0, reset_applied, invalid};
    }

    const double dt = std::min(s.dt_sec, config_.control_dt_max_sec);
    const bool enter_recovery =
        std::abs(s.signed_cross_track_m) >= config_.recovery_enter_cross_track_m ||
        std::abs(s.heading_error_rad) >= config_.recovery_enter_heading_error_rad ||
        (config_.recovery_enter_on_bearing_clamp && s.bearing_clamped);
    const bool inside_exit_gates =
        std::abs(s.signed_cross_track_m) <= config_.recovery_exit_cross_track_m &&
        std::abs(s.heading_error_rad) <= config_.recovery_exit_heading_error_rad &&
        !s.bearing_clamped;

    std::string reason;
    if (enter_recovery) {
      reason = entry_reason(s);
      state_ = TrackingControlState::recovery;
      stable_dwell_sec_ = 0.0;
    } else if (state_ == TrackingControlState::tracking) {
      reason = "tracking_held";
    } else if (inside_exit_gates) {
      if (state_ == TrackingControlState::recovery) {
        state_ = TrackingControlState::recapture_stable;
        reason = "recapture_dwell_started";
      } else {
        reason = "recapture_dwell_held";
      }
      stable_dwell_sec_ += dt;
      if (stable_dwell_sec_ >= config_.stable_recapture_dwell_sec) {
        stable_dwell_sec_ = config_.stable_recapture_dwell_sec;
        state_ = TrackingControlState::tracking;
        reason = "stable_recapture_complete";
      }
    } else {
      state_ = TrackingControlState::recovery;
      stable_dwell_sec_ = 0.0;
      reason = "recapture_exit_gate_lost";
    }
    if (reset_applied &&
        (reason == "recapture_dwell_started" || reason == "recapture_dwell_held")) {
      reason = "path_reset_recapture";
    }

    const bool acceleration_allowed = state_ == TrackingControlState::tracking;
    const double speed_scale = acceleration_allowed ? 1.0 :
        state_ == TrackingControlState::recapture_stable ?
        config_.recapture_speed_scale : config_.recovery_speed_scale;
    return {state_, true, acceleration_allowed, speed_scale,
            reason, dt, stable_dwell_sec_, reset_applied, std::nullopt};
  }

 private:
  static std::optional<std::string> invalid_reason(const TrackingControlInput &s) {
    // Preserve the Python reference's stale-telemetry precedence.
    if (!s.telemetry_fresh) return std::string("stale_telemetry");
    for (const auto &value : {
        std::pair<const char *, double>{"projection_s_m", s.projection_s_m},
        {"signed_cross_track_m", s.signed_cross_track_m},
        {"heading_error_rad", s.heading_error_rad},
        {"commanded_speed_mps", s.commanded_speed_mps},
        {"measured_speed_mps", s.measured_speed_mps},
        {"dt_sec", s.dt_sec}}) {
      if (!std::isfinite(value.second))
        return std::string("nonfinite_") + value.first;
    }
    if (s.projection_s_m < 0.0) return std::string("negative_projection_s_m");
    if (s.commanded_speed_mps < 0.0) return std::string("negative_commanded_speed_mps");
    if (s.measured_speed_mps < 0.0) return std::string("negative_measured_speed_mps");
    if (s.dt_sec < 0.0) return std::string("negative_dt_sec");
    return std::nullopt;
  }

  std::string entry_reason(const TrackingControlInput &s) const {
    std::string result = "recovery_entered:";
    const auto append = [&](const char *name) {
      if (result != "recovery_entered:") result += "+";
      result += name;
    };
    if (std::abs(s.signed_cross_track_m) >= config_.recovery_enter_cross_track_m)
      append("cross_track");
    if (std::abs(s.heading_error_rad) >= config_.recovery_enter_heading_error_rad)
      append("heading");
    if (config_.recovery_enter_on_bearing_clamp && s.bearing_clamped)
      append("bearing_clamp");
    return result;
  }

  TrackingControlConfig config_;
  TrackingControlState state_{TrackingControlState::recapture_stable};
  double stable_dwell_sec_{0.0};
  std::optional<std::string> path_identity_{};
};

}  // namespace dyx::rpp
