#pragma once
// D2C: Pure C++17 reverse-to-pivot sequencing shadow. No actuator authority.
// This adapter reuses the existing verified pivot FSM. The distinct reverse
// alignment must be certified before pivot is requested. A failed/stale gate
// permanently latches HOLD until an externally validated new run constructs
// a fresh shadow instance. Synthetic test evidence does not authorize PX4.
#include "dyx_rpp_cpp17_core/pivot_motion.hpp"
#include "dyx_rpp_cpp17_core/reverse_pivot_handoff.hpp"
#include <cmath>
#include <string>

namespace dyx::rpp {
struct PivotSequenceShadowInput {
  ReversePivotHandoffResult handoff{};
  double now_sec{0.0};
  double dt_sec{0.0};
  double pivot_centre_radial_error_m{0.0};
  double measured_linear_speed_mps{0.0};
  double measured_yaw_rate_radps{0.0};
  double heading_error_rad{0.0};
  bool trusted_mission_owner{false};
  bool stop_and_mark_certified{false};
  bool position_and_yaw_fresh{false};
  bool rear_and_pivot_calibrated{false};
  bool physical_safety_verified{false};
  bool reverse_alignment_certified{false};
  bool recapture_certified{false};
};
struct PivotSequenceShadowResult {
  bool eligible{false};
  bool hold_zero{true};
  bool transition_attempted{false};
  bool failed{false};
  bool reverse_needed{false};
  bool release_certified{false};
  bool motion_authorized{false}; // Permanently false: diagnostic only.
  MotionState state{MotionState::TRACK};
  MotionDirective directive{MotionDirective::HOLD_ZERO};
  const char* reason{"NOT_EVALUATED"};
};
class NativePivotSequenceShadow final {
 public:
  PivotSequenceShadowResult step(const PivotSequenceShadowInput& in) noexcept {
    PivotSequenceShadowResult out{};
    out.state = machine_.state();
    if (latched_) { out.reason="SHADOW_LATCHED"; out.failed=true; return out; }
    if (!finite(in) || in.now_sec < 0.0 || in.dt_sec < 0.0 ||
        in.dt_sec > 0.10 || in.pivot_centre_radial_error_m < 0.0 ||
        (started_ && in.now_sec <= last_time_)) {
      latched_=true; out.reason="INVALID_SAMPLE"; out.failed=true; return out;
    }
    // STOP and physical safety are NOT inferred from ROS status or RTK FIX.
    if (!in.trusted_mission_owner || !in.stop_and_mark_certified ||
        !in.position_and_yaw_fresh || !in.rear_and_pivot_calibrated ||
        !in.physical_safety_verified || !in.handoff.geometry_valid ||
        !in.handoff.action_authorized) {
      // D2A currently never sets action_authorized; so the ROS adapter is
      // deliberately unable to advance even with apparently healthy GNSS.
      out.reason="UNCOMMISSIONED_HOLD";
      return out;
    }
    if (in.handoff.phase == HandoffPhase::HOLD) {
      out.reason="HANDOFF_HOLD"; return out;
    }
    if (in.handoff.phase == HandoffPhase::DIRECT_FORWARD) {
      out.eligible=true; out.reason="DIRECT_FORWARD_DIAGNOSTIC_ONLY";
      return out;
    }
    out.reverse_needed = in.handoff.phase == HandoffPhase::REVERSE_TO_PIVOT;
    if (out.reverse_needed && !in.reverse_alignment_certified) {
      out.reason="REVERSE_ALIGNMENT_NOT_CERTIFIED";
      return out;
    }
    if (in.handoff.phase != HandoffPhase::STATIONARY_PIVOT && !out.reverse_needed) {
      out.reason="UNSUPPORTED_PHASE"; return out;
    }
    // An actual pivot only follows a certified reverse alignment, never a
    // D2A distance calculation. The shadow *never* outputs commands.
    try {
      PivotMotionInput sample{};
      sample.monotonic_time_sec=in.now_sec;
      sample.dt_sec=in.dt_sec;
      sample.anchor_radial_error_m=in.pivot_centre_radial_error_m;
      sample.measured_linear_speed_mps=in.measured_linear_speed_mps;
      sample.measured_yaw_rate_radps=in.measured_yaw_rate_radps;
      sample.heading_error_rad=in.heading_error_rad;
      sample.telemetry_fresh=in.position_and_yaw_fresh;
      sample.pivot_requested=true;
      sample.brake_to_anchor_requested=true;
      sample.recapture_complete=in.recapture_certified;
      const auto state=machine_.step(sample);
      out.eligible=true;
      out.transition_attempted=true;
      out.state=state.state;
      out.directive=MotionDirective::HOLD_ZERO; // Never export directive to DDS.
      out.release_certified=state.release_certificate.valid;
      out.failed=state.failed;
      out.reason=state.failed ? "PIVOT_FSM_FAILED" : "PIVOT_SHADOW_ONLY";
      if (state.failed) latched_=true;
      started_=true;
      last_time_=in.now_sec;
    } catch (...) {
      latched_=true; out.failed=true; out.reason="PIVOT_FSM_EXCEPTION";
    }
    return out;
  }
 private:
  static bool finite(const PivotSequenceShadowInput& s) noexcept {
    return std::isfinite(s.now_sec) && std::isfinite(s.dt_sec) &&
      std::isfinite(s.pivot_centre_radial_error_m) &&
      std::isfinite(s.measured_linear_speed_mps) &&
      std::isfinite(s.measured_yaw_rate_radps) &&
      std::isfinite(s.heading_error_rad);
  }
  VerifiedPivotStateMachine machine_{};
  bool started_{false};
  bool latched_{false};
  double last_time_{-1.0};
};
} // namespace dyx::rpp
