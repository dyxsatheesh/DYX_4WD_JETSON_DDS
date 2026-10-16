#pragma once

#include "dyx_rpp_cpp17_core/mission_command_gateway.hpp"
#include "dyx_rpp_cpp17_core/native_mission_rpp_pipeline.hpp"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace dyx::rpp {

// One process-local owner for mission and forward motion decisions.
// No public API accepts a caller-fabricated DDS mission safety tick.
// Does not emit ROS messages or hardware commands.
struct NativeOwnedControlOutput {
  DdsMissionRuntimeOutput mission{};
  NativeRppExecutionOutput motion{};
};

class NativeControlOwner final {
 public:
  NativeControlOwner() = default;

  NativeControlOwner(const NativeControlOwner&) = delete;
  NativeControlOwner& operator=(const NativeControlOwner&) = delete;

  ClientCommandStatus load(
      const std::string& run_id,
      const std::string& session_id,
      std::vector<MarkingGoal> goals,
      MissionMode mode,
      bool trusted_upload_authorized,
      bool trusted_control_lease,
      bool trusted_px4_disarmed) {
    return gateway_.load(
        run_id, session_id, std::move(goals), mode,
        trusted_upload_authorized,
        trusted_control_lease,
        trusted_px4_disarmed);
  }

  DdsTelemetryDecision observe_battery(const DdsBatteryFrame& f) {
    return gateway_.observe_battery(f);
  }

  DdsTelemetryDecision observe_gps(const DdsGpsFrame& f) {
    return gateway_.observe_gps(f);
  }

  DdsTelemetryDecision observe_vehicle(const DdsVehicleFrame& f) {
    return gateway_.observe_vehicle(f);
  }

  DdsTelemetryDecision observe_estimator(const DdsEstimatorFrame& f) {
    return gateway_.observe_estimator(f);
  }

  DdsTelemetryDecision observe_corrections(const DdsCorrectionsFrame& f) {
    return gateway_.observe_corrections(f);
  }

  DdsTelemetryDecision observe_obstacle(const DdsObstacleFrame& f) {
    return gateway_.observe_obstacle(f);
  }

  ClientCommandResult dispatch(const ClientCommandRequest& cmd) {
    return gateway_.dispatch(cmd);
  }

  // Recovery facts must come from authenticated, independently
  // verified sources. Never expose recovery as an ordinary ROS
  // parameter or unauthenticated topic.
  bool begin_new_disarmed_run(
      bool trusted_authorization,
      bool verified_px4_disarmed) {
    if (!trusted_authorization || !verified_px4_disarmed ||
        gateway_.recovery_quarantined())
      return false;

    return pipeline_.begin_new_disarmed_run(
        trusted_authorization, verified_px4_disarmed);
  }

  NativeOwnedControlOutput tick(
      double host_now_sec,
      bool mission_session_active,
      bool operator_motion_enabled,
      bool physical_emergency_stop,
      MissionSequenceInput mission_evidence,
      NativeMissionRppInput guidance_request) {

    // Force a single monotonic control-tick timestamp.
    mission_evidence.monotonic_time_sec = host_now_sec;

    NativeOwnedControlOutput out;

    // Safety is evaluated by the actual gateway using its own
    // previously ingested PX4 and independent safety telemetry.
    out.mission = gateway_.tick(
        host_now_sec,
        mission_session_active,
        operator_motion_enabled,
        physical_emergency_stop,
        std::move(mission_evidence));

    // Caller must not authorize motion just by presenting guidance.
    if (!gateway_.has_active_mission() ||
        gateway_.safe_stop_requested() ||
        gateway_.recovery_quarantined() ||
        physical_emergency_stop ||
        !mission_session_active ||
        !operator_motion_enabled) {
      guidance_request.guidance.motion_enabled = false;
    }

    out.motion = pipeline_.step(
        out.mission, guidance_request);

    return out;
  }

  void force_stop() {
    gateway_.force_stop();
  }

  void quarantine() {
    gateway_.latch_recovery_quarantine();
  }

  const MissionSequencer& mission() const {
    return gateway_.mission();
  }

  bool has_active_mission() const {
    return gateway_.has_active_mission();
  }

  bool safe_stop_requested() const {
    return gateway_.safe_stop_requested();
  }

  bool hardware_command_emitted() const {
    return false;
  }

 private:
  NativeMissionCommandGateway gateway_{};
  NativeMissionRppPipeline pipeline_{};
};

} // namespace dyx::rpp
