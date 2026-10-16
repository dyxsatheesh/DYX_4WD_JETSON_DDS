#pragma once

#include "dyx_rpp_cpp17_core/mission_command_gateway.hpp"
#include "dyx_rpp_cpp17_core/native_mission_rpp_pipeline.hpp"
#include "dyx_rpp_cpp17_core/trajectory_builder.hpp"
#include "dyx_rpp_cpp17_core/mission_runtime_install.hpp"
#include "dyx_rpp_cpp17_core/mission_leg_preview.hpp"

#include <cstdint>
#include <memory>
#include <stdexcept>
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

  // Production geometry-to-mission binding. Authorization/disarm
  // attestations MUST come from a trusted backend/PX4 adapter.
  ClientCommandStatus load_prepared(
      const std::string& run_id,
      const std::string& session_id,
      const std::vector<Point2D>& marking_points,
      MissionMode mode,
      bool upload_authorized,
      bool control_lease_verified,
      bool px4_disarmed_verified,
      TrajectoryBuildConfig trajectory_cfg = {}) {

    if (!upload_authorized || !control_lease_verified ||
        !px4_disarmed_verified)
      return ClientCommandStatus::unauthorized;

    if (prepared_ || gateway_.has_active_mission())
      return ClientCommandStatus::invalid_stage;

    try {
      // Keep the trajectory and semantic goals in the same
      // immutable prepared-path snapshot.
      auto trajectory =
          NativeTrajectoryBuilder(trajectory_cfg).build(marking_points);

      auto snapshot =
          signed_snapshot(trajectory.navigation, marking_points);

      auto candidate = std::make_unique<NativeMissionInstallation>(
          std::move(snapshot), run_id, mode);

      auto status = gateway_.load(
          run_id, session_id, candidate->marking_goals, mode,
          upload_authorized, control_lease_verified,
          px4_disarmed_verified);

      if (status != ClientCommandStatus::accepted)
        return status;

      prepared_ = std::move(candidate);
      return ClientCommandStatus::accepted;

    } catch (const std::exception&) {
      return ClientCommandStatus::runtime_rejected;
    }
  }

  const NativeMissionInstallation* prepared_mission() const {
    return prepared_.get();
  }

  // Diagnostic-only active-leg calculation. Neither this function
  // nor its output can authorize PX4 motor or spray commands.
  //
  // P1 is intentionally excluded: the first marking point has
  // no inbound mission segment, and must be acquired/certified
  // by a separate initial-position state machine.
  MissionLegPreviewResult preview_active_leg(
      const MissionLegPreviewInput& input) {
    MissionLegPreviewResult denied{};

    if (!prepared_ || !gateway_.has_active_mission() ||
        gateway_.safe_stop_requested() ||
        gateway_.recovery_quarantined()) {
      denied.reason = "NO_ACTIVE_PREPARED_MISSION";
      return denied;
    }

    const auto& status = gateway_.mission().status();

    if (status.stage != MissionStage::approaching ||
        status.current_marking >= prepared_->marking_goals.size()) {
      denied.reason = "MISSION_NOT_APPROACHING";
      return denied;
    }

    if (status.current_marking == 0) {
      denied.reason = "P1_INITIAL_ACQUISITION_REQUIRED";
      return denied;
    }

    const auto& goal =
        prepared_->marking_goals.at(status.current_marking);

    if (status.active_terminal_identity != goal.terminal_identity) {
      denied.reason = "ACTIVE_GOAL_IDENTITY_MISMATCH";
      return denied;
    }

    if (active_preview_marking_ != status.current_marking) {
      // A previously latched preview must not silently restart.
      // A verified disarmed recovery boundary is required.
      if (active_preview_ && active_preview_->latched()) {
        denied.reason = "PREVIEW_RECOVERY_REQUIRED";
        return denied;
      }

      try {
        auto candidate = std::make_unique<MissionLegPreview>(
            prepared_->snapshot.raw_navigation(),
            goal.raw_path_index);
        if (candidate->goal_identity() != goal.goal_label) {
          denied.reason = "PREVIEW_GOAL_LABEL_MISMATCH";
          return denied;
        }
        active_preview_ = std::move(candidate);
        active_preview_marking_ = status.current_marking;
      } catch (const std::exception&) {
        denied.reason = "PREVIEW_CONSTRUCTION_FAILED";
        return denied;
      }
    }

    // A caller-controlled boolean is not evidence of calibration.
    // This result is diagnostic only until calibration and
    // authenticated safety adapters are implemented.
    return active_preview_->step(input);
  }

  void invalidate_preview() {
    active_preview_.reset();
    active_preview_marking_ = 0;
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

    const bool accepted = pipeline_.begin_new_disarmed_run(
        trusted_authorization, verified_px4_disarmed);

    if (accepted) invalidate_preview();
    return accepted;
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
    invalidate_preview();
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
  std::unique_ptr<NativeMissionInstallation> prepared_{};
  NativeMissionRppPipeline pipeline_{};
  std::unique_ptr<MissionLegPreview> active_preview_{};
  std::size_t active_preview_marking_{0};
};

} // namespace dyx::rpp
