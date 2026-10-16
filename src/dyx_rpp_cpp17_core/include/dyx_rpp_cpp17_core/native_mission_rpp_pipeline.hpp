#pragma once

#include "dyx_rpp_cpp17_core/dds_mission_runtime.hpp"
#include "dyx_rpp_cpp17_core/native_rpp_executor.hpp"

#include <cmath>
#include <cstdint>

namespace dyx::rpp {

// Non-actuating mission-to-RPP decision pipeline.
//
// The DDS mission tick MUST originate from the same trusted runtime
// and host monotonic control cycle as the guidance input.
//
// No ROS publisher or physical actuator is implemented here.
struct NativeMissionRppInput {
  std::uint64_t sequence{0};
  ForwardYawInput guidance{};
  bool trusted_operator_lease{false};
  bool trusted_estop_released{false};
  bool calibrated_marking_reference{false};
};

class NativeMissionRppPipeline final {
 public:
  bool begin_new_disarmed_run(
      bool trusted_authorized,
      bool verified_px4_disarmed) {
    return executor_.begin_new_disarmed_run(
        trusted_authorized, verified_px4_disarmed);
  }

  NativeRppExecutionOutput step(
      const DdsMissionRuntimeOutput& mission_tick,
      const NativeMissionRppInput& request) {

    const auto& t = mission_tick.telemetry;
    const auto& s = t.safety;
    const auto& decision = mission_tick.decision;

    // Never accept a mission decision from an earlier/future tick.
    const bool clock_matches =
        std::isfinite(request.guidance.now_sec) &&
        std::isfinite(s.now_sec) &&
        request.guidance.now_sec >= 0.0 &&
        std::abs(request.guidance.now_sec - s.now_sec) <= 0.001;

    const bool mission_authorized =
        clock_matches &&
        !mission_tick.telemetry_fault_latched &&
        !t.telemetry_integrity_fault &&
        mission_tick.preview_forward_permitted &&
        decision.preview_forward_permitted &&
        decision.safety.motion_allowed &&
        decision.safety.hold == SafetyHold::none &&
        !decision.safety.manual_ack_required &&
        decision.mission.stage == MissionStage::approaching &&
        !decision.mission.hold_zero;

    MotionSafetyEvidence safety{};

    safety.mission_running = mission_authorized;
    safety.safety_authorized = mission_authorized;

    safety.rtk_fixed =
        t.gps_sample_fresh &&
        s.gnss_fix_type == 6;

    safety.battery_ok =
        t.battery_sample_fresh &&
        std::isfinite(s.battery_voltage_v) &&
        s.battery_voltage_v >= 22.0;

    safety.estimator_valid = s.estimator_valid;

    safety.obstacle_clear =
        std::isfinite(s.obstacle_age_sec) &&
        s.obstacle_age_sec >= 0.0 &&
        s.obstacle_age_sec <= 0.5 &&
        s.obstacle_clear;

    safety.operator_lease_valid =
        request.trusted_operator_lease;

    safety.physical_estop_released =
        request.trusted_estop_released;

    safety.marking_reference_calibrated =
        request.calibrated_marking_reference;

    NativeRppExecutionInput execution{};
    execution.sequence = request.sequence;
    execution.guidance = request.guidance;
    execution.safety = safety;

    // A rejected mission decision must also latch the RPP gate.
    // No automatic restart on return to RTK FIXED.
    return executor_.step(execution);
  }

  bool active() const {
    return executor_.active();
  }

 private:
  NativeRppExecutor executor_{};
};

}  // namespace dyx::rpp
