#pragma once
// D2D: mission-bound, NON-ACTUATING terminal-stop diagnostic.
// Independent RAW GNSS surveying and physical safety must still authorize marking.
#include "dyx_rpp_cpp17_core/terminal_stop.hpp"
#include <cmath>
#include <cstdint>
#include <string>

namespace dyx::rpp {
struct TerminalShadowEvidence {
  double now_sec{0.0}, position_sample_sec{0.0};
  double along_m{0.0}, cross_m{0.0};
  double east_m{0.0}, north_m{0.0};
  double position_speed_mps{0.0}, yaw_rate_radps{0.0};
  double requested_speed_mps{0.0};
  bool mission_approaching{false};
  bool telemetry_fresh{false};
  bool reference_calibrated{false};
  bool trusted_rtk_fixed{false};
  bool physical_safety_verified{false};
  std::string goal_identity{};
};
struct TerminalShadowDecision {
  const char* reason{"NO_TRUSTED_TERMINAL_EVIDENCE"};
  bool eligible{false};
  bool stopped_in_estimator_frame{false};
  bool hold_zero{true};
  bool physical_command_authorized{false};
  bool spray_authorized{false};
  double radial_error_m{0.0};
  double stopping_distance_m{0.0};
};
class NativeTerminalShadow final {
 public:
  TerminalShadowDecision observe(const TerminalShadowEvidence& e) noexcept {
    TerminalShadowDecision d{};
    if (!std::isfinite(e.along_m) || !std::isfinite(e.cross_m)) {
      d.reason = "NONFINITE_POSITION"; return d;
    }
    d.radial_error_m = std::hypot(e.along_m, e.cross_m);
    if (!std::isfinite(d.radial_error_m)) {
      d.reason = "OVERFLOW_POSITION"; return d;
    }
    const bool ready = e.mission_approaching && e.telemetry_fresh &&
      e.reference_calibrated && e.trusted_rtk_fixed &&
      e.physical_safety_verified && !e.goal_identity.empty();
    if (!ready) { d.reason = "SAFETY_GATE_UNVERIFIED"; return d; }
    if (latched_) { d.reason = "TERMINAL_LATCHED"; return d; }
    if (!std::isfinite(e.now_sec) || !std::isfinite(e.position_sample_sec) ||
        !std::isfinite(e.east_m) || !std::isfinite(e.north_m) ||
        !std::isfinite(e.position_speed_mps) || !std::isfinite(e.yaw_rate_radps) ||
        !std::isfinite(e.requested_speed_mps) || e.position_speed_mps < 0.0 ||
        e.requested_speed_mps < 0.0 || e.now_sec < 0.0 ||
        e.position_sample_sec < 0.0 || e.position_sample_sec > e.now_sec ||
        e.now_sec - e.position_sample_sec > 0.20) {
      latched_ = true; d.reason = "INVALID_OR_STALE_SAMPLE"; return d;
    }
    if (!goal_.empty() && goal_ != e.goal_identity) {
      latched_ = true; d.reason = "GOAL_IDENTITY_CHANGED"; return d;
    }
    if (goal_.empty()) goal_ = e.goal_identity;
    RadialStopInput in{};
    in.monotonic_time_sec = e.now_sec;
    in.position_sample_time_sec = e.position_sample_sec;
    in.active = true;
    in.terminal_identity = e.goal_identity;
    in.along_remaining_m = e.along_m;
    in.cross_error_m = e.cross_m;
    in.position_x_m = e.east_m;
    in.position_y_m = e.north_m;
    in.position_derived_speed_mps = e.position_speed_mps;
    in.measured_yaw_rate_radps = e.yaw_rate_radps;
    in.tracking_speed_command_mps = e.requested_speed_mps;
    in.telemetry_fresh = true;
    try {
      const auto result = regulator_.step(in);
      d.eligible = true;
      d.stopping_distance_m = result.stop_distance_m;
      d.hold_zero = result.hold_zero;
      d.stopped_in_estimator_frame = result.certificate.has_value() &&
        result.certificate->terminal_identity == e.goal_identity &&
        result.certificate->radial_error_m <= 0.020;
      d.reason = d.stopped_in_estimator_frame ? "ESTIMATOR_STOP_ONLY" :
        (result.failure != RadialStopFailure::none ? "STOP_FAILED" : "AWAIT_STOP");
      if (result.failure != RadialStopFailure::none) latched_ = true;
    } catch (...) {
      latched_ = true; d.reason = "TERMINAL_EXCEPTION";
    }
    return d;
  }
  bool latched() const noexcept { return latched_; }
 private:
  TerminalStopRegulator regulator_{};
  std::string goal_{};
  bool latched_{false};
};
}  // namespace dyx::rpp
