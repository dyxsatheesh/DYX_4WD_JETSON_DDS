#pragma once
// Phase 2H4B: ROS-free, NON-ACTUATING DDS telemetry -> safety -> mission runtime.
// Does NOT publish setpoints, issue VehicleCommand, control motors or tools.
// All timestamps supplied by the caller use ONE host monotonic clock.
// Each independently received telemetry update revokes prior command authority
// until a NEW same-clock safety/mission step has been evaluated.

#include "dyx_rpp_cpp17_core/dds_safety_telemetry.hpp"
#include "dyx_rpp_cpp17_core/safety_mission_bridge.hpp"
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dyx::rpp {

struct DdsMissionRuntimeOutput {
  DdsSafetySnapshot telemetry{};
  SafetyMissionBridgeOutput decision{};
  bool telemetry_fault_latched{false};
  bool preview_forward_permitted{false}; // advisory for THIS tick ONLY
  bool physical_command_emitted{false}; // invariant: always false
};

class NativeDdsMissionRuntime final {
 public:
  explicit NativeDdsMissionRuntime(
      DdsTelemetryConfig telemetry_cfg = {},
      SafetyConfig safety_cfg = {},
      MissionSequenceConfig mission_cfg = {},
      SafetyMissionBridgeConfig bridge_cfg = {})
      : telemetry_(telemetry_cfg),
        bridge_(safety_cfg, mission_cfg, bridge_cfg) {}

  DdsTelemetryDecision observe_battery(const DdsBatteryFrame &v) {
    return ingest(telemetry_.observe_battery(v));
  }
  DdsTelemetryDecision observe_gps(const DdsGpsFrame &v) {
    return ingest(telemetry_.observe_gps(v));
  }
  DdsTelemetryDecision observe_vehicle(const DdsVehicleFrame &v) {
    return ingest(telemetry_.observe_vehicle(v));
  }
  DdsTelemetryDecision observe_estimator(const DdsEstimatorFrame &v) {
    return ingest(telemetry_.observe_estimator(v));
  }
  DdsTelemetryDecision observe_corrections(const DdsCorrectionsFrame &v) {
    return ingest(telemetry_.observe_corrections(v));
  }
  DdsTelemetryDecision observe_obstacle(const DdsObstacleFrame &v) {
    return ingest(telemetry_.observe_obstacle(v));
  }

  // Upload authorization and disarmed state must be verified by the trusted
  // operator/PX4 adapter; a caller-supplied bool is NOT cryptographic proof.
  void load(std::vector<MarkingGoal> goals, MissionMode mode,
            bool upload_authorized, bool px4_disarmed) {
    if (telemetry_fault_latched_ || telemetry_.integrity_fault_latched())
      throw std::logic_error("telemetry epoch fault: cannot load mission");
    bridge_.load(std::move(goals), mode, upload_authorized, px4_disarmed);
    mission_loaded_ = true;
    last_evaluated_revision_.reset();
  }

  // Neither this function nor its result can be used to actuate hardware.
  // Call at the deterministic mission-control tick; NEVER cache the advisory
  // preview beyond the tick. The mission evidence must use the same clock.
  DdsMissionRuntimeOutput step(double now_sec, bool mission_session_active,
                                bool operator_motion_enabled,
                                bool physical_emergency_stop,
                                MissionSequenceInput mission_evidence) {
    DdsMissionRuntimeOutput out;
    out.telemetry = telemetry_.snapshot(now_sec, mission_session_active,
                                        operator_motion_enabled,
                                        physical_emergency_stop);
    if (out.telemetry.telemetry_integrity_fault) {
      latch_telemetry_fault();
    }
    out.decision = bridge_.step(out.telemetry.safety, std::move(mission_evidence));
    last_evaluated_revision_ = telemetry_revision_;
    out.telemetry_fault_latched = telemetry_fault_latched_;
    out.preview_forward_permitted = !telemetry_fault_latched_ &&
        mission_loaded_ && out.decision.preview_forward_permitted &&
        !out.telemetry.telemetry_integrity_fault;
    return out;
  }

  // Re-observing ANY telemetry topic requires another safety tick before an
  // operator command. This closes the gap between a danger update and tick.
  bool start(double now_sec) {
    return action_evidence_current() && bridge_.start(now_sec);
  }
  bool resume_manually(double now_sec) {
    return action_evidence_current() && bridge_.resume_manually(now_sec);
  }
  bool next(double now_sec) {
    return action_evidence_current() && bridge_.next(now_sec);
  }
  bool skip(double now_sec) {
    return action_evidence_current() && bridge_.skip(now_sec);
  }
  void pause() {
    bridge_.pause();
    last_evaluated_revision_.reset();
  }
  // STOP must always be available even with bad time, RTK, battery, PX4,
  // obstacle or journal state. No re-START without a fresh load.
  void stop() {
    bridge_.stop();
    mission_loaded_ = false;
    last_evaluated_revision_.reset();
  }

  bool acknowledge_manual_hold(bool authorized, bool px4_disarmed) {
    last_evaluated_revision_.reset();
    return !telemetry_fault_latched_ &&
        bridge_.acknowledge_manual_hold(authorized, px4_disarmed);
  }
  bool acknowledge_battery_replacement(bool authorized, bool px4_disarmed) {
    last_evaluated_revision_.reset();
    return !telemetry_fault_latched_ &&
        bridge_.acknowledge_battery_replacement(authorized, px4_disarmed);
  }
  bool release_emergency_stop(bool authorized, bool px4_disarmed,
                              bool physical_estop_released) {
    last_evaluated_revision_.reset();
    return !telemetry_fault_latched_ &&
        bridge_.release_emergency_stop(authorized, px4_disarmed,
                                       physical_estop_released);
  }

  // A PX4 reboot/source change/invalid telemetry epoch terminates the mission.
  // Recovery must be an EXPLICIT, verified disarmed operation. It cannot resume
  // the previous mission and it cannot manufacture fresh sensor evidence.
  bool begin_new_disarmed_px4_epoch(bool authorized, bool px4_disarmed) {
    if (!telemetry_.begin_new_disarmed_px4_epoch(authorized, px4_disarmed))
      return false;
    stop();
    telemetry_fault_latched_ = false;
    ++telemetry_revision_;
    return true;
  }

  bool telemetry_fault_latched() const { return telemetry_fault_latched_; }
  const MissionSequencer &mission() const { return bridge_.mission(); }
  bool has_active_mission() const { return mission_loaded_; }

 private:
  DdsTelemetryDecision ingest(DdsTelemetryDecision result) {
    if (telemetry_revision_ == std::numeric_limits<std::uint64_t>::max()) {
      latch_telemetry_fault();
      return DdsTelemetryDecision::fault_latched;
    }
    ++telemetry_revision_;
    last_evaluated_revision_.reset();
    if (result == DdsTelemetryDecision::invalid_data ||
        result == DdsTelemetryDecision::clock_regression ||
        result == DdsTelemetryDecision::px4_epoch_changed ||
        result == DdsTelemetryDecision::fault_latched)
      latch_telemetry_fault();
    return result;
  }
  void latch_telemetry_fault() {
    telemetry_fault_latched_ = true;
    stop();
  }
  bool action_evidence_current() const {
    return mission_loaded_ && !telemetry_fault_latched_ &&
        !telemetry_.integrity_fault_latched() &&
        last_evaluated_revision_ &&
        *last_evaluated_revision_ == telemetry_revision_;
  }
  NativeDdsSafetyTelemetry telemetry_;
  NativeSafetyMissionBridge bridge_;
  std::uint64_t telemetry_revision_{0};
  std::optional<std::uint64_t> last_evaluated_revision_{};
  bool telemetry_fault_latched_{false};
  bool mission_loaded_{false};
};

} // namespace dyx::rpp
