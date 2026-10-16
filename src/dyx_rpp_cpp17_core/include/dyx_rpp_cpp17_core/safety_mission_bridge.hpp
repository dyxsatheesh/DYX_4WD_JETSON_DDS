#pragma once
// Phase 2H1B: NON-ACTUATING safety-to-mission integration, C++17 only.
// Combines the native arbiter and mission sequencer. This class NEVER publishes
// PX4 setpoints, arms/disarms, changes modes, operates spray, or drives wheels.
// The runtime adapter must supply monotonic and validated per-source telemetry.
// H8F: RTK FLOAT immediately inhibits autonomous mission progression.
// RTK FIXED recovery requires explicit manual RESUME after fresh safety checks.

#include "dyx_rpp_cpp17_core/mission_sequence.hpp"
#include "dyx_rpp_cpp17_core/safety_arbiter.hpp"
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dyx::rpp {

struct SafetyMissionBridgeConfig {
  double action_gate_fresh_sec{0.25};
  double clock_agreement_sec{0.001};
  void validate() const {
    if (!std::isfinite(action_gate_fresh_sec) ||
        action_gate_fresh_sec <= 0.0 || action_gate_fresh_sec > 1.0 ||
        !std::isfinite(clock_agreement_sec) ||
        clock_agreement_sec < 0.0 || clock_agreement_sec > 0.01)
      throw std::invalid_argument("invalid safety-mission bridge time gate");
  }
};

struct SafetyMissionBridgeOutput {
  SafetyOutput safety{};
  MissionSequenceOutput mission{};
  // Advisory ONLY; not a request to command PX4 or any actuator.
  bool preview_forward_permitted{false};
  bool float_pause_owned{false};
  bool auto_resumed_float{false};
  bool rc_allowed{true};
  bool telemetry_required{true};
  bool stop_allowed{true};
  std::string bridge_reason{"NO_SAFETY_SAMPLE"};
};

class NativeSafetyMissionBridge final {
 public:
  explicit NativeSafetyMissionBridge(
      SafetyConfig safety_cfg = {}, MissionSequenceConfig mission_cfg = {},
      SafetyMissionBridgeConfig bridge_cfg = {})
      : safety_(safety_cfg), mission_(mission_cfg), bridge_cfg_(bridge_cfg) {
    bridge_cfg_.validate();
  }

  // Upload/load is an explicitly authorized, disarmed operation. Do not allow
  // a different snapshot to overwrite READY/APPROACHING/PAUSED/etc.
  void load(std::vector<MarkingGoal> goals, MissionMode mode,
            bool upload_authorized, bool px4_disarmed) {
    if (!upload_authorized || !px4_disarmed)
      throw std::logic_error("mission load requires authorized disarmed state");
    const auto stage = mission_.stage();
    if (stage != MissionStage::unloaded && stage != MissionStage::stopped &&
        stage != MissionStage::finished && stage != MissionStage::fault)
      throw std::logic_error("cannot replace active or ready mission");
    mission_.load(std::move(goals), mode);
    float_pause_owned_ = false;
    last_frame_.reset();
  }

  // The calling runtime passes the exact safety input used by the arbiter and
  // the mission evidence for the SAME host monotonic tick. The input's
  // mission_running flag means an authorized mission session exists, including
  // a temporarily paused session; it is not a PX4 motor enable bit.
  SafetyMissionBridgeOutput step(const SafetyInput &s, MissionSequenceInput m) {
    const SafetyOutput safety = safety_.step(s);
    const bool matching_clock = std::isfinite(s.now_sec) &&
        std::isfinite(m.monotonic_time_sec) &&
        std::abs(s.now_sec - m.monotonic_time_sec) <= bridge_cfg_.clock_agreement_sec;
    // Do not reuse a prior valid gate after a bad clock or mismatched evidence.
    if (!matching_clock || safety.hold == SafetyHold::bad_clock) {
      last_frame_.reset();
      float_pause_owned_ = false;
      mission_.stop();
      return make(safety, false, "INCONSISTENT_MONOTONIC_TIMES_STOPPED");
    }
    last_frame_ = LastFrame{s.now_sec, safety};
    const bool arbiter_estop = safety.hold == SafetyHold::emergency_stop;
    m.emergency_stop = m.emergency_stop || arbiter_estop;
    m.motion_health_authorized = safety.motion_allowed && !m.emergency_stop;

    if (m.emergency_stop) {
      float_pause_owned_ = false;
      return make(safety, mission_.step(m), false, "EMERGENCY_STOP_LATCHED");
    }

    const MissionStage stage_before = mission_.stage();
    if (!safety.motion_allowed) {
      if (stage_before == MissionStage::approaching ||
          stage_before == MissionStage::verify_hold ||
          stage_before == MissionStage::spray_ack ||
          stage_before == MissionStage::resolved) {
        mission_.pause();
        float_pause_owned_ = false;      } else if (safety.hold != SafetyHold::rtk_float_paused) {
        // E.g. FLOAT pause followed by obstacle, missing PX4, dead RTK, or
        // operator-disable: revoke the special auto-resume permission.
        float_pause_owned_ = false;
      }
      return make(safety, mission_.step(m), false, "SAFETY_HOLD");
    }

    // H8F: Returning RTK FIXED never resumes a paused mission.
    // The operator must explicitly invoke resume_manually() using a
    // new, positive safety decision.
    const auto result = mission_.step(m);
    // Sequencer's preview permission can only be asserted in APPROACHING and
    // only with a fresh positive arbiter sample and no emergency stop.
    return make(safety, result, false, "SAFETY_VERIFIED");
  }

  // All movement-authorizing operator commands require a fresh positive
  // safety observation. PAUSE and STOP are always permitted.
  void pause() {
    mission_.pause();
    float_pause_owned_ = false;  // operator PAUSE is never auto-resumable
  }
  bool start(double now_sec) {
    if (!fresh_authority(now_sec) || mission_.stage() != MissionStage::ready)
      return false;
    mission_.start(true);
    return true;
  }
  bool resume_manually(double now_sec) {
    if (!fresh_authority(now_sec) || float_pause_owned_ ||
        mission_.stage() != MissionStage::paused)
      return false;
    mission_.resume(true);
    return true;
  }
  bool next(double now_sec) {
    if (!fresh_authority(now_sec) || mission_.stage() != MissionStage::await_next)
      return false;
    mission_.next();
    return true;
  }
  bool skip(double now_sec) {
    if (!fresh_authority(now_sec) || mission_.stage() != MissionStage::approaching)
      return false;
    mission_.skip();
    return true;
  }
  void stop() {
    mission_.stop();
    float_pause_owned_ = false;
    last_frame_.reset();
  }
  bool acknowledge_manual_hold(bool authorized, bool px4_disarmed) {
    last_frame_.reset();
    return safety_.acknowledge_manual_hold(authorized, px4_disarmed);
  }
  bool acknowledge_battery_replacement(bool authorized, bool px4_disarmed) {
    last_frame_.reset();
    return safety_.acknowledge_battery_replacement(authorized, px4_disarmed);
  }
  bool release_emergency_stop(bool authorized, bool px4_disarmed,
                              bool physical_estop_released) {
    last_frame_.reset();
    return safety_.release_emergency_stop(authorized, px4_disarmed,
                                         physical_estop_released);
  }
  const MissionSequencer &mission() const { return mission_; }
  bool float_pause_owned() const { return float_pause_owned_; }

 private:
  struct LastFrame { double now_sec{}; SafetyOutput safety{}; };
  bool fresh_authority(double now_sec) const {
    if (!last_frame_ || !std::isfinite(now_sec) || now_sec < last_frame_->now_sec ||
        now_sec - last_frame_->now_sec > bridge_cfg_.action_gate_fresh_sec)
      return false;
    return last_frame_->safety.motion_allowed &&
           last_frame_->safety.start_resume_next_skip_allowed &&
           !last_frame_->safety.manual_ack_required &&
           last_frame_->safety.hold == SafetyHold::none;
  }
  SafetyMissionBridgeOutput make(SafetyOutput safety, bool auto_resumed,
                                  const char *reason) const {
    return make(safety, mission_.status(), auto_resumed, reason);
  }
  SafetyMissionBridgeOutput make(SafetyOutput safety, MissionSequenceOutput mission,
                                  bool auto_resumed, const char *reason) const {
    SafetyMissionBridgeOutput o;
    o.safety = safety;
    o.mission = std::move(mission);
    o.preview_forward_permitted = safety.motion_allowed &&
        safety.hold == SafetyHold::none && !safety.manual_ack_required &&
        o.mission.allow_next_leg_preview && !o.mission.hold_zero &&
        o.mission.stage == MissionStage::approaching;
    o.float_pause_owned = float_pause_owned_;
    o.auto_resumed_float = auto_resumed;
    o.rc_allowed = safety.rc_allowed;
    o.telemetry_required = safety.telemetry_must_remain_live;
    o.stop_allowed = safety.stop_allowed;
    o.bridge_reason = reason;
    return o;
  }
  NativeSafetyArbiter safety_;
  MissionSequencer mission_;
  SafetyMissionBridgeConfig bridge_cfg_;
  std::optional<LastFrame> last_frame_{};
  bool float_pause_owned_{false};
};

} // namespace dyx::rpp
