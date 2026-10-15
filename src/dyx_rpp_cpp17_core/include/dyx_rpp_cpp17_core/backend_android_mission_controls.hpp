#pragma once
// Phase 2H4N: OFFLINE C++17 model of Android mission-command UI proposals.
// NO actual Android client, HTTP request, identity verification, sockets, DDS,
// PX4/motor/laser/spray control, or permission to move. This contract consumes
// already checked *in-process* backend/telemetry views. A real Android app
// must independently implement authenticated TLS, strict JSON, and lifecycle.
// UI hints are NEVER authorization: the backend must check every command.

#include "dyx_rpp_cpp17_core/backend_android_telemetry_contract.hpp"
#include "dyx_rpp_cpp17_core/backend_mission_api.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace dyx::rpp {

enum class AndroidMissionIntent { start, pause, resume, next, skip, stop };

inline const char* android_control_route(AndroidMissionIntent intent) {
  switch (intent) {
    case AndroidMissionIntent::start: return kBackendMissionStartPath;
    case AndroidMissionIntent::pause: return kBackendMissionPausePath;
    case AndroidMissionIntent::resume: return kBackendMissionResumePath;
    case AndroidMissionIntent::next: return kBackendMissionNextPath;
    case AndroidMissionIntent::skip: return kBackendMissionSkipPath;
    case AndroidMissionIntent::stop: return kBackendMissionStopPath;
  }
  return "";
}

struct AndroidControlScreenConfig {
  double max_mission_status_age_sec{2.0};
  double response_timeout_sec{2.0};
  void validate() const {
    if (!std::isfinite(max_mission_status_age_sec) ||
        max_mission_status_age_sec < 0.1 || max_mission_status_age_sec > 30.0 ||
        !std::isfinite(response_timeout_sec) || response_timeout_sec < 0.1 ||
        response_timeout_sec > 30.0)
      throw std::invalid_argument("invalid offline Android mission controls limits");
  }
};

struct AndroidControlScreenContext {
  std::string mission_run_id{};
  // Status is assumed to have come from the already authenticated backend,
  // not from Android JSON. All booleans below are UI hints, NEVER grants.
  BackendMissionApiResponse mission{};
  double mission_status_received_sec{0.0}; // HOST monotonic observation time
  BackendAndroidReplicaView live{};
  bool ui_signed_in{false};
  bool ui_control_lease_visible{false};
  bool ui_network_connected{false};
  bool unsaved_mission_edits{false};
  bool operator_confirmed_skip{false};
};

struct AndroidControlDraft {
  std::string code{"ANDROID_CONTEXT_REQUIRED"};
  std::string method{};
  std::string path{};
  std::string json_body{}; // strictly bounded two-field JSON, not yet sent
  std::uint64_t sequence{0};
  bool prepared{false};
  bool requires_backend_authorization{true};
  bool network_io_performed{false};
  bool hardware_command_emitted{false};
};

struct AndroidControlAcknowledgment {
  std::string code{"ANDROID_NO_PENDING_PROPOSAL"};
  bool backend_decision_reported{false};
  bool executed_on_rover{false}; // NEVER infer actuation from HTTP 200
  bool network_io_performed{false};
  bool hardware_command_emitted{false};
};

class NativeOfflineAndroidMissionControls final {
 public:
  explicit NativeOfflineAndroidMissionControls(AndroidControlScreenConfig cfg = {})
      : cfg_(cfg) { cfg_.validate(); }

  // Neither status nor any UI flag is a cryptographic proof. If run identity
  // changes, fence any pending reply; a stale approval cannot cross missions.
  void show(AndroidControlScreenContext context) {
    const bool changed = have_context_ &&
        (context.mission_run_id != ctx_.mission_run_id ||
         context.ui_signed_in != ctx_.ui_signed_in ||
         context.ui_network_connected != ctx_.ui_network_connected ||
         context.ui_control_lease_visible != ctx_.ui_control_lease_visible);
    if (changed || !context.ui_network_connected ||
        context.live.requires_resync || !context.live.connected)
      pending_ = false;
    // A trusted status indicating recovery quarantine fences pending motion
    // proposals. A later healthy-looking status must never clear this latch.
    if (context.mission.recovery_quarantined) {
      recovery_quarantine_latched_ = true;
      pending_ = false;
    }
    ctx_ = std::move(context);
    have_context_ = true;
  }

  // STOP is always a possible proposal for a signed-in operator, including
  // FLOAT, battery hold, missing telemetry, no lease and an inflight command.
  // It still needs a valid run ID + a network path and a separately verified
  // backend. Actual physical emergency stop MUST be independent of HTTP.
  AndroidControlDraft propose(AndroidMissionIntent intent, double trusted_host_sec) {
    const bool is_stop = intent == AndroidMissionIntent::stop;
    if (!have_context_) return denied("ANDROID_CONTEXT_REQUIRED");
    if (!ctx_.ui_signed_in) return denied("ANDROID_UI_SIGN_IN_REQUIRED");
    if (!valid_native_mission_run_id(ctx_.mission_run_id))
      return denied("ANDROID_MISSION_ID_REQUIRED");
    if (!ctx_.ui_network_connected)
      return denied(is_stop ? "ANDROID_STOP_DELIVERY_UNAVAILABLE" :
                              "ANDROID_NETWORK_UNAVAILABLE");
    if (recovery_quarantine_latched_ && !is_stop)
      return denied("ANDROID_RECOVERY_QUARANTINED");

    // Clock faults never obstruct preparation of the STOP advisory request.
    if (!check_clock(trusted_host_sec) && !is_stop)
      return denied("ANDROID_HOST_CLOCK_FAULT_LATCHED");
    if (fault_ && !is_stop) return denied("ANDROID_HOST_CLOCK_FAULT_LATCHED");
    if (sequence_ == std::numeric_limits<std::uint64_t>::max()) {
      sequence_fault_ = true;
      return denied("ANDROID_SEQUENCE_EXHAUSTED");
    }
    if (is_stop) {
      // No UI state (including another unresolved request) can block STOP.
      pending_=false;
    } else {
      if (pending_) return denied("ANDROID_REQUEST_PENDING");
      if (!ctx_.ui_control_lease_visible)
        return denied("ANDROID_UI_CONTROL_LEASE_REQUIRED");
      if (!status_current(trusted_host_sec))
        return denied("ANDROID_FRESH_MISSION_STATUS_REQUIRED");
      if (ctx_.mission.safe_stop_intent)
        return denied("ANDROID_BACKEND_SAFE_STOP_INTENT");
      if (!stage_allows(intent)) return denied("ANDROID_INVALID_MISSION_STAGE");
      if (ctx_.unsaved_mission_edits)
        return denied("ANDROID_UNSAVED_MISSION_EDITS");
      if (intent == AndroidMissionIntent::skip && !ctx_.operator_confirmed_skip)
        return denied("ANDROID_SKIP_CONFIRMATION_REQUIRED");
      if (requires_fresh_rtk(intent) &&
          (!ctx_.live.connected || ctx_.live.requires_resync ||
           !ctx_.live.snapshot_fresh || !ctx_.live.rtk_fixed ||
           ctx_.live.hardware_command_emitted || ctx_.live.network_io_performed))
        return denied("ANDROID_FRESH_RTK_FIXED_REQUIRED");
    }
    const auto seq=++sequence_;
    pending_ = true;
    pending_sequence_ = seq;
    pending_intent_ = intent;
    pending_since_ = trusted_host_sec;
    pending_run_ = ctx_.mission_run_id;
    AndroidControlDraft draft;
    draft.code=is_stop ? "ANDROID_STOP_PROPOSAL_UNCONFIRMED" :
                         "ANDROID_REQUEST_PROPOSAL_NOT_SENT";
    draft.method="POST";
    draft.path=android_control_route(intent);
    draft.json_body="{\"mission_run_id\":\"" + ctx_.mission_run_id +
                    "\",\"sequence\":" + std::to_string(seq) + "}";
    draft.sequence=seq;
    draft.prepared=true;
    return draft;
  }

  // Caller MUST independently verify TLS, identity, response correlation,
  // replay controls and time. Even a 200 + accepted decision means only that
  // the backend decision was accepted; it is NOT a physical execution ACK.
  AndroidControlAcknowledgment observe_backend(std::uint64_t request_sequence,
      const BackendMissionApiResponse& response, double trusted_host_sec) {
    if (!pending_ || request_sequence != pending_sequence_ ||
        !have_context_ || ctx_.mission_run_id != pending_run_)
      return ack("ANDROID_REPLY_UNMATCHED_OR_REPLAYED");
    const bool is_stop=pending_intent_==AndroidMissionIntent::stop;
    if (!check_clock(trusted_host_sec) && !is_stop) {
      pending_=false;
      return ack("ANDROID_HOST_CLOCK_FAULT_LATCHED");
    }
    if (!is_stop && trusted_host_sec-pending_since_>cfg_.response_timeout_sec) {
      pending_=false;
      return ack("ANDROID_REPLY_EXPIRED_RESULT_UNKNOWN");
    }
    pending_=false;
    if (response.hardware_command_emitted || !response.decision_only)
      return ack("ANDROID_UNSAFE_REPLY_REJECTED");
    if (response.http_status == 423 &&
        response.code == "MISSION_RECOVERY_QUARANTINED") {
      recovery_quarantine_latched_ = true;
      return {"ANDROID_RECOVERY_QUARANTINED",true,false,false,false};
    }
    if (response.http_status == 200 && response.code=="MISSION_DECISION_ACCEPTED")
      return {"ANDROID_BACKEND_DECISION_ACCEPTED_NOT_ACTUATED",true,false,false,false};
    return {"ANDROID_BACKEND_DECISION_DENIED_OR_UNKNOWN",true,false,false,false};
  }

  AndroidControlAcknowledgment expire(double trusted_host_sec) {
    if (!check_clock(trusted_host_sec)) {
      pending_=false;
      return ack("ANDROID_HOST_CLOCK_FAULT_LATCHED");
    }
    if (!pending_) return ack("ANDROID_NO_PENDING_PROPOSAL");
    if (trusted_host_sec-pending_since_>cfg_.response_timeout_sec) {
      pending_=false;
      return ack("ANDROID_REQUEST_TIMED_OUT_RESULT_UNKNOWN");
    }
    return ack("ANDROID_AWAITING_BACKEND_DECISION");
  }

  void disconnected() {
    pending_=false;
    ctx_.ui_network_connected=false;
    ctx_.live.connected=false;
    ctx_.live.requires_resync=true;
    ctx_.live.rtk_fixed=false;
  }

  // Only a NEW instance/session may reset the lifetime sequence. Clock-fault
  // recovery still requires real external operator approval and PX4 DISARMED.
  bool recover_disarmed(bool independently_authorized_operator,
                        bool independently_confirmed_px4_disarmed,
                        double trusted_new_host_sec) {
    if (!independently_authorized_operator || !independently_confirmed_px4_disarmed ||
        !std::isfinite(trusted_new_host_sec) || trusted_new_host_sec < 0.0 ||
        sequence_fault_) return false;
    fault_=false;
    last_clock_sec_=trusted_new_host_sec;
    pending_=false;
    have_context_=false;
    ctx_={};
    return true;
  }

  bool request_pending() const { return pending_; }
  bool recovery_quarantine_latched() const { return recovery_quarantine_latched_; }
  bool clock_fault_latched() const { return fault_; }
  std::uint64_t last_proposed_sequence() const { return sequence_; }
 private:
  static AndroidControlDraft denied(const char* code) { AndroidControlDraft r; r.code=code; return r; }
  static AndroidControlAcknowledgment ack(const char* code) {
    AndroidControlAcknowledgment r; r.code=code; return r;
  }
  bool check_clock(double t) {
    if (fault_) return false;
    if (!std::isfinite(t) || t<0.0 || t<last_clock_sec_) {
      fault_=true;pending_=false;return false;
    }
    last_clock_sec_=t;
    return true;
  }
  bool status_current(double t) const {
    const auto& s=ctx_.mission;
    if (!std::isfinite(ctx_.mission_status_received_sec) ||
        ctx_.mission_status_received_sec<0.0 ||
        ctx_.mission_status_received_sec>t ||
        t-ctx_.mission_status_received_sec>cfg_.max_mission_status_age_sec ||
        s.http_status!=200 || s.code!="STATUS_SNAPSHOT_READ_ONLY" ||
        s.stage_name!=mission_stage_name(s.stage) || !s.mission_loaded ||
        s.total==0 || s.completed>s.total || s.failed>s.total ||
        s.skipped>s.total || s.completed>s.total-s.failed ||
        s.skipped>s.total-s.completed-s.failed) return false;
    return true;
  }
  bool stage_allows(AndroidMissionIntent intent) const {
    switch (intent) {
      case AndroidMissionIntent::start: return ctx_.mission.stage==MissionStage::ready;
      case AndroidMissionIntent::pause:
        return ctx_.mission.stage==MissionStage::ready ||
               ctx_.mission.stage==MissionStage::approaching ||
               ctx_.mission.stage==MissionStage::verify_hold ||
               ctx_.mission.stage==MissionStage::resolved ||
               ctx_.mission.stage==MissionStage::await_next;
      case AndroidMissionIntent::resume: return ctx_.mission.stage==MissionStage::paused;
      case AndroidMissionIntent::next:
      case AndroidMissionIntent::skip:
        return ctx_.mission.stage==MissionStage::resolved ||
               ctx_.mission.stage==MissionStage::await_next;
      case AndroidMissionIntent::stop: return true;
    }
    return false;
  }
  static bool requires_fresh_rtk(AndroidMissionIntent intent) {
    return intent==AndroidMissionIntent::start || intent==AndroidMissionIntent::resume ||
           intent==AndroidMissionIntent::next || intent==AndroidMissionIntent::skip;
  }
  AndroidControlScreenConfig cfg_{};
  AndroidControlScreenContext ctx_{};
  bool have_context_{false};
  bool pending_{false};
  bool fault_{false};
  bool sequence_fault_{false};
  bool recovery_quarantine_latched_{false};
  AndroidMissionIntent pending_intent_{AndroidMissionIntent::start};
  std::uint64_t sequence_{0};
  std::uint64_t pending_sequence_{0};
  double pending_since_{0.0};
  double last_clock_sec_{0.0};
  std::string pending_run_{};
};

} // namespace dyx::rpp
