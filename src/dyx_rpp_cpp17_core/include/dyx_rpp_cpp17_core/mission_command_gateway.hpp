#pragma once
// Phase 2H4C: NON-ACTUATING C++17 client mission-command safety boundary.
// Trusted authentication, lease, monotonic clock and disarm facts MUST be
// supplied by an independently verified backend ingress, never user JSON.
// No network listener, ROS publisher, PX4 command or motor/spray action.

#include "dyx_rpp_cpp17_core/dds_mission_runtime.hpp"
#include "dyx_rpp_cpp17_core/mission_runtime_install.hpp" // run ID validation
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dyx::rpp {

enum class ClientMissionAction { start, pause, resume, next, skip, stop };
enum class ClientCommandStatus {
  accepted, unauthorized, invalid_identity, wrong_run, no_mission,
  duplicate_or_reordered, invalid_clock, stale_or_consumed_safety,
  invalid_stage, runtime_rejected, run_id_reused
};
inline const char *client_command_status_name(ClientCommandStatus s) {
  switch(s) {
    case ClientCommandStatus::accepted: return "ACCEPTED_DECISION_ONLY";
    case ClientCommandStatus::unauthorized: return "UNAUTHORIZED";
    case ClientCommandStatus::invalid_identity: return "INVALID_IDENTITY";
    case ClientCommandStatus::wrong_run: return "WRONG_MISSION_RUN";
    case ClientCommandStatus::no_mission: return "NO_ACTIVE_MISSION";
    case ClientCommandStatus::duplicate_or_reordered: return "DUPLICATE_OR_REORDERED";
    case ClientCommandStatus::invalid_clock: return "INVALID_HOST_CLOCK";
    case ClientCommandStatus::stale_or_consumed_safety: return "FRESH_SAFETY_TICK_REQUIRED";
    case ClientCommandStatus::invalid_stage: return "INVALID_MISSION_STAGE";
    case ClientCommandStatus::runtime_rejected: return "RUNTIME_REJECTED";
    case ClientCommandStatus::run_id_reused: return "RUN_ID_REUSED";
  }
  return "INVALID_STATUS";
}

struct ClientCommandRequest {
  ClientMissionAction action{ClientMissionAction::stop};
  std::string mission_run_id{};
  std::string operator_session_id{};
  std::uint64_t session_sequence{0};
  // This is a trusted HOST steady-clock receive time, not Android wall time.
  double received_at_host_sec{std::numeric_limits<double>::quiet_NaN()};
  // These are already VERIFIED by a privileged backend authorization adapter.
  bool authenticated_operator{false};
  bool authorized_control_lease{false};
};
struct ClientCommandResult {
  ClientCommandStatus status{ClientCommandStatus::unauthorized};
  MissionStage stage{MissionStage::unloaded};
  bool accepted() const {return status == ClientCommandStatus::accepted;}
  bool hardware_command_emitted{false}; // invariant: always false
  bool safe_stop_intent{false};          // advisory, not a PX4 safe stop
};

class NativeMissionCommandGateway final {
 public:
  // Constructor does not start any mission or ROS transport.
  explicit NativeMissionCommandGateway(
      DdsTelemetryConfig telemetry_cfg={}, SafetyConfig safety_cfg={},
      MissionSequenceConfig mission_cfg={}, SafetyMissionBridgeConfig bridge_cfg={})
      : runtime_(telemetry_cfg, safety_cfg, mission_cfg, bridge_cfg) {}

  // Loading accepts a PREVALIDATED mission goal list from the trusted signed
  // snapshot installer. The bools are external attestations, not authentication.
  ClientCommandStatus load(const std::string &run_id,
                           const std::string &operator_session_id,
                           std::vector<MarkingGoal> goals, MissionMode mode,
                           bool trusted_upload_authorized,
                           bool trusted_control_lease,
                           bool trusted_px4_disarmed) {
    if (!trusted_upload_authorized || !trusted_control_lease ||
        !trusted_px4_disarmed) return ClientCommandStatus::unauthorized;
    if (!valid_native_mission_run_id(run_id) ||
        !valid_native_mission_run_id(operator_session_id))
      return ClientCommandStatus::invalid_identity;
    if (used_run_ids_.count(run_id)) return ClientCommandStatus::run_id_reused;
    if (runtime_.has_active_mission()) return ClientCommandStatus::invalid_stage;
    try {
      runtime_.load(std::move(goals), mode, true, true);
    } catch (const std::exception &) {
      return ClientCommandStatus::runtime_rejected;
    }
    used_run_ids_.insert(run_id);
    active_run_ = run_id;
    active_session_ = operator_session_id;
    highest_sequence_.reset();
    last_command_time_.reset();
    control_tick_consumed_=true; // First START requires a fresh healthy tick.
    stop_requested_=false;
    return ClientCommandStatus::accepted;
  }

  // Every incoming telemetry event invalidates the underlying runtime's
  // positive gate, even if the frame is rejected. Never reuse previous tick.
  DdsTelemetryDecision observe_battery(const DdsBatteryFrame &f){return ingest(runtime_.observe_battery(f));}
  DdsTelemetryDecision observe_gps(const DdsGpsFrame &f){return ingest(runtime_.observe_gps(f));}
  DdsTelemetryDecision observe_vehicle(const DdsVehicleFrame &f){return ingest(runtime_.observe_vehicle(f));}
  DdsTelemetryDecision observe_estimator(const DdsEstimatorFrame &f){return ingest(runtime_.observe_estimator(f));}
  DdsTelemetryDecision observe_corrections(const DdsCorrectionsFrame &f){return ingest(runtime_.observe_corrections(f));}
  DdsTelemetryDecision observe_obstacle(const DdsObstacleFrame &f){return ingest(runtime_.observe_obstacle(f));}

  DdsMissionRuntimeOutput tick(double host_now_sec,
                               bool mission_session_active,
                               bool operator_motion_enabled,
                               bool physical_emergency_stop,
                               MissionSequenceInput evidence) {
    auto result=runtime_.step(host_now_sec,mission_session_active,
                              operator_motion_enabled,physical_emergency_stop,
                              std::move(evidence));
    // The runtime will independently reject bad telemetry and clock mismatch.
    control_tick_consumed_=false;
    if (runtime_.telemetry_fault_latched() || !runtime_.has_active_mission()) {
      active_run_.clear(); active_session_.clear();
      control_tick_consumed_=true;
      stop_requested_=true; // advisory safe-stop, not physical stop proof
    }
    return result;
  }

  ClientCommandResult dispatch(const ClientCommandRequest &request){
    // STOP does not require healthy telemetry, OFFBOARD, a fresh control tick,
    // or a valid time. It DOES require an authenticated operator session;
    // the separate unconditional force_stop() is for safety hardware/software.
    if (!request.authenticated_operator) return reply(ClientCommandStatus::unauthorized);
    if (!valid_native_mission_run_id(request.operator_session_id) ||
        !valid_native_mission_run_id(request.mission_run_id) ||
        request.session_sequence==0) return reply(ClientCommandStatus::invalid_identity);
    if (!runtime_.has_active_mission()) {
      if (request.action==ClientMissionAction::stop) {
        safe_stop(); // idempotent STOP despite missing/stale telemetry
        return reply(ClientCommandStatus::accepted);
      }
      return reply(ClientCommandStatus::no_mission);
    }
    if (request.mission_run_id != active_run_ ||
        request.operator_session_id != active_session_)
      return reply(ClientCommandStatus::wrong_run);
    if (highest_sequence_ && request.session_sequence <= *highest_sequence_)
      return reply(ClientCommandStatus::duplicate_or_reordered);
    // STOP is intentionally allowed with a failed/stale host time.
    if (request.action==ClientMissionAction::stop){
      highest_sequence_=request.session_sequence;
      safe_stop();
      return reply(ClientCommandStatus::accepted);
    }
    if (!request.authorized_control_lease)
      return reply(ClientCommandStatus::unauthorized);
    if (!std::isfinite(request.received_at_host_sec) ||
        request.received_at_host_sec < 0.0 ||
        (last_command_time_ && request.received_at_host_sec < *last_command_time_)) {
      return reply(ClientCommandStatus::invalid_clock);
    }
    // Consume well-authenticated monotonically sequenced commands even when
    // unsafe; a replay cannot become valid after the environment changes.
    highest_sequence_=request.session_sequence;
    last_command_time_=request.received_at_host_sec;
    const MissionStage stage=runtime_.mission().stage();
    if (request.action==ClientMissionAction::pause) {
      if (stage!=MissionStage::approaching &&
          stage!=MissionStage::verify_hold &&
          stage!=MissionStage::resolved &&
          stage!=MissionStage::spray_ack)
        return reply(ClientCommandStatus::invalid_stage);
      runtime_.pause();
      control_tick_consumed_=true;
      return reply(ClientCommandStatus::accepted);
    }
    if (control_tick_consumed_)
      return reply(ClientCommandStatus::stale_or_consumed_safety);
    if ((request.action==ClientMissionAction::start && stage!=MissionStage::ready) ||
        (request.action==ClientMissionAction::resume && stage!=MissionStage::paused) ||
        (request.action==ClientMissionAction::next && stage!=MissionStage::await_next) ||
        (request.action==ClientMissionAction::skip && stage!=MissionStage::approaching))
      return reply(ClientCommandStatus::invalid_stage);
    bool success=false;
    switch(request.action){
      case ClientMissionAction::start: success=runtime_.start(request.received_at_host_sec);break;
      case ClientMissionAction::resume: success=runtime_.resume_manually(request.received_at_host_sec);break;
      case ClientMissionAction::next: success=runtime_.next(request.received_at_host_sec);break;
      case ClientMissionAction::skip: success=runtime_.skip(request.received_at_host_sec);break;
      default: return reply(ClientCommandStatus::invalid_stage);
    }
    if (!success) return reply(ClientCommandStatus::runtime_rejected);
    control_tick_consumed_=true; // No command chaining on one safety tick.
    return reply(ClientCommandStatus::accepted);
  }

  // Safety equipment and watchdog adapters have an unconditional decision-side
  // STOP; the physical motor/actuator stop must be implemented separately.
  void force_stop() {safe_stop();}
  bool begin_new_disarmed_px4_epoch(bool trusted_authorized,
                                    bool trusted_px4_disarmed){
    const bool success=runtime_.begin_new_disarmed_px4_epoch(
        trusted_authorized,trusted_px4_disarmed);
    if (success) {
      active_run_.clear(); active_session_.clear();
      highest_sequence_.reset();last_command_time_.reset();
      control_tick_consumed_=true;stop_requested_=true;
    }
    return success;
  }
  const MissionSequencer &mission() const {return runtime_.mission();}
  bool safe_stop_requested() const {return stop_requested_;}
  bool has_active_mission() const {return runtime_.has_active_mission();}
  bool emits_hardware_commands() const {return false;}

 private:
  DdsTelemetryDecision ingest(DdsTelemetryDecision decision) {
    // The runtime latches and stops immediately on a corrupt/new PX4 epoch;
    // do not defer exposure of the safe-stop intent until the next tick.
    if (runtime_.telemetry_fault_latched()) safe_stop();
    return decision;
  }
  ClientCommandResult reply(ClientCommandStatus status) const {
    ClientCommandResult out;
    out.status=status;
    out.stage=runtime_.mission().stage();
    out.safe_stop_intent=stop_requested_;
    return out;
  }
  void safe_stop(){
    runtime_.stop();
    active_run_.clear();active_session_.clear();
    highest_sequence_.reset();last_command_time_.reset();
    control_tick_consumed_=true;stop_requested_=true;
  }
  NativeDdsMissionRuntime runtime_{};
  std::set<std::string> used_run_ids_{};
  std::string active_run_{},active_session_{};
  std::optional<std::uint64_t> highest_sequence_{};
  std::optional<double> last_command_time_{};
  bool control_tick_consumed_{true},stop_requested_{false};
};

} // namespace dyx::rpp
