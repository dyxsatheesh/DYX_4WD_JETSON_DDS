#pragma once

#include "dyx_rpp_cpp17_core/forward_yaw_rate.hpp"
#include "dyx_rpp_cpp17_core/motion_command_gate.hpp"

#include <cmath>
#include <cstdint>

namespace dyx::rpp {

struct NativeRppExecutionInput {
  std::uint64_t sequence{0};
  ForwardYawInput guidance{};
  MotionSafetyEvidence safety{};
};

struct NativeRppExecutionOutput {
  bool permitted{false};
  bool latched{true};
  double north_mps{0.0};
  double east_mps{0.0};
  double yaw_rate_enu_radps{0.0};
  ForwardYawPhase phase{ForwardYawPhase::SAFE_HOLD};
};

class NativeRppExecutor final {
 public:
  // External authentication and PX4 disarm checks are mandatory.
  // These parameters are attestations from a trusted owner, not proof.
  bool begin_new_disarmed_run(bool authorized, bool px4_disarmed) {
    if (!gate_.reset_for_new_run(authorized, px4_disarmed))
      return false;
    planner_.reset();
    active_ = true;
    return true;
  }

  NativeRppExecutionOutput step(
      const NativeRppExecutionInput& in) {

    NativeRppExecutionOutput out{};

    if (!active_ || gate_.latched())
      return out;

    auto safety = in.safety;

    safety.safety_authorized =
        safety.safety_authorized &&
        in.guidance.motion_enabled &&
        in.guidance.telemetry_fresh &&
        !in.guidance.stop_requested;

    // The planner and gate consume the same control tick.
    const auto guidance = planner_.step(in.guidance);

    safety.safety_authorized =
        safety.safety_authorized && !guidance.safe_hold;

    MotionDemand demand{};
    demand.host_time_sec = in.guidance.now_sec;
    demand.sequence = in.sequence;
    demand.forward_mps = guidance.forward_speed_mps;
    demand.yaw_rate_enu_radps = guidance.yaw_rate_enu_radps;
    demand.current_yaw_enu_rad = in.guidance.rover_yaw_enu_rad;
    demand.safety = safety;

    const auto accepted = gate_.evaluate(demand);

    out.latched = gate_.latched();
    out.phase = guidance.phase;

    if (!accepted.permitted || guidance.safe_hold) {
      active_ = false;
      return out;
    }

    // Belt-and-braces consistency check: motion must remain
    // aligned with the current rover heading.
    constexpr double tolerance = 1e-8;

    if (std::abs(
            accepted.velocity_north_mps -
            guidance.velocity_north_mps) > tolerance ||
        std::abs(
            accepted.velocity_east_mps -
            guidance.velocity_east_mps) > tolerance ||
        std::abs(
            accepted.yaw_rate_enu_radps -
            guidance.yaw_rate_enu_radps) > tolerance) {
      active_ = false;
      return {};
    }

    out.permitted = true;
    out.latched = false;
    out.north_mps = accepted.velocity_north_mps;
    out.east_mps = accepted.velocity_east_mps;
    out.yaw_rate_enu_radps = accepted.yaw_rate_enu_radps;

    return out;
  }

  bool active() const {
    return active_ && !gate_.latched();
  }

 private:
  ForwardYawRatePlanner planner_{};
  MotionCommandGate gate_{};
  bool active_{false};
};

}  // namespace dyx::rpp
