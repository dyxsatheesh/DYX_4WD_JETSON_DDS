#pragma once
// DYX Autonomous Phase 2H5K: offline start-once PREVIEW-only ingress guard.
// Demonstrates an explicit backend lifecycle boundary. NOT wired to a real
// HTTP listener or all existing backend entry points. The caller MUST prevent
// bypass access to the underlying backend/gateway and supply independently
// authenticated operator/session facts. No PX4 / DDS / actuator I/O.

#include "dyx_rpp_cpp17_core/backend_draft_quarantined_review_lab.hpp"
#include <cstdint>
#include <string>
#include <utility>

namespace dyx::rpp {
enum class DraftQuarantinedBootGuardLabState {
  new_instance, read_only_preview, locked_permanently
};

struct DraftQuarantinedBootGuardLabResult {
  DraftQuarantinedBootGuardLabState state{
      DraftQuarantinedBootGuardLabState::new_instance};
  DraftQuarantinedReviewLabState audit{
      DraftQuarantinedReviewLabState::denied};
  bool can_read_preview{false};
  bool can_upload_or_update{false};
  bool can_load{false};
  bool can_start_or_resume{false};
  bool can_unquarantine{false};
  bool hardware_command_emitted{false};
};

class NativeOfflineQuarantinedBootGuardLab final {
 public:
  NativeOfflineQuarantinedBootGuardLab(
      std::string file, NativeMissionCommandGateway& gateway,
      NativeBackendMissionUpload& backend)
      : review_(std::move(file)), gateway_(gateway), backend_(backend) {}
  NativeOfflineQuarantinedBootGuardLab(
      const NativeOfflineQuarantinedBootGuardLab&) = delete;
  NativeOfflineQuarantinedBootGuardLab& operator=(
      const NativeOfflineQuarantinedBootGuardLab&) = delete;

  // ONE admission attempt per process instance. A failed attempt requires a
  // brand-new instance; retrying must not turn a failed boot into authority.
  DraftQuarantinedBootGuardLabResult begin(
      const DraftQuarantinedReviewLabRequest& request) {
    if (state_!=DraftQuarantinedBootGuardLabState::new_instance)
      return make_result(DraftQuarantinedReviewLabState::denied);
    state_=DraftQuarantinedBootGuardLabState::locked_permanently;
    // First reject an already-active live backend BEFORE latching STOP so
    // a stopped active mission can never masquerade as a clean startup.
    if (gateway_.has_active_mission() || backend_.has_draft()) {
      backend_.latch_recovery_quarantine();
      return make_result(DraftQuarantinedReviewLabState::backend_active);
    }
    // This closes direct backend.handle()/gateway.load()/dispatch() bypasses
    // using THESE SAME objects. A separate fresh gateway remains a separate
    // authority; production must enforce exclusive construction/ownership.
    backend_.latch_recovery_quarantine();
    const auto audit=review_.inspect(request,gateway_,backend_);
    if (audit.state!=DraftQuarantinedReviewLabState::reviewed_preview_only ||
        !audit.read_only_snapshot || !audit.mission_mode)
      return make_result(audit.state);
    pin_.operator_session_id=request.current_identity.operator_session_id;
    pin_.epoch=request.current_identity.session_epoch;
    pin_.mission_run_id=audit.mission_run_id;
    pin_.signature=audit.signature;
    pin_.nonce=audit.nonce;
    pin_.generation=audit.generation;
    pin_.revision=audit.revision;
    pin_.last_sequence=audit.last_sequence;
    state_=DraftQuarantinedBootGuardLabState::read_only_preview;
    return make_result(audit.state,true);
  }

  // Always re-read the *entire* durable H5J evidence. No cached coordinates
  // or signed flags are treated as current proof of a safe mission. If any
  // later recheck fails, this process instance becomes permanently locked.
  DraftQuarantinedReviewLabResult inspect_current(
      const DraftQuarantinedReviewLabRequest& request) {
    DraftQuarantinedReviewLabResult denied;
    if (state_!=DraftQuarantinedBootGuardLabState::read_only_preview)
      return denied;
    const auto& id=request.current_identity;
    if (!request.explicit_operator_review || !id.authenticated ||
        !id.can_read_mission || !id.fresh_session_evidence ||
        id.operator_session_id!=pin_.operator_session_id ||
        id.session_epoch!=pin_.epoch) return denied;
    auto refreshed=review_.inspect(request,gateway_,backend_);
    if (refreshed.state!=DraftQuarantinedReviewLabState::reviewed_preview_only ||
        !refreshed.read_only_snapshot || !refreshed.mission_mode ||
        refreshed.mission_run_id!=pin_.mission_run_id ||
        refreshed.signature!=pin_.signature ||
        refreshed.quarantined_epoch!=pin_.epoch ||
        refreshed.nonce!=pin_.nonce ||
        refreshed.generation!=pin_.generation ||
        refreshed.revision!=pin_.revision ||
        refreshed.last_sequence!=pin_.last_sequence) {
      retire();
      return denied;
    }
    return refreshed;
  }

  // This covers only the adapter's offered upload/draft API surface. The
  // production ingress router must make this the SOLE path when quarantined;
  // a direct call to NativeBackendMissionUpload::handle bypasses this lab.
  // Independent emergency STOP must NOT be routed through this adapter.
  BackendMissionUploadResponse handle(
      const BackendMissionUploadRequest& request,
      const DraftQuarantinedReviewLabRequest& review_request) {
    BackendMissionUploadResponse out;
    out.http_status=423;
    out.code="RECOVERY_QUARANTINE_COMMANDS_LOCKED";
    if (request.path!=kBackendMissionDraftPath || request.method!="GET")
      return out;
    if (state_!=DraftQuarantinedBootGuardLabState::read_only_preview)
      return out;
    // Fail closed for GET-with-body, mixed read/write and replay fields.
    if (!request.marking_points.empty() || request.sequence!=0 ||
        request.expected_revision!=0 || !request.expected_signature.empty())
      return out;
    if (!request.principal.authenticated ||
        !request.principal.can_read_mission ||
        request.principal.operator_session_id!=pin_.operator_session_id ||
        request.mission_run_id!=pin_.mission_run_id)
      return out;
    const auto refreshed=inspect_current(review_request);
    if (refreshed.state!=DraftQuarantinedReviewLabState::reviewed_preview_only ||
        !refreshed.read_only_snapshot) return out;
    out.http_status=200;
    out.code="QUARANTINED_PREVIEW_READ_ONLY";
    out.revision=refreshed.revision;
    out.signature=refreshed.signature;
    out.marking_count=refreshed.read_only_snapshot->marking_points.size();
    out.navigation_count=refreshed.read_only_snapshot->navigation_points.size();
    out.draft_ready=true;
    out.loaded=false;
    out.preview_only=true;
    out.hardware_command_emitted=false;
    return out;
  }

  // Irreversible for this instance; it never authorizes new commands.
  void retire() {
    state_=DraftQuarantinedBootGuardLabState::locked_permanently;
    backend_.latch_recovery_quarantine();
    pin_=Pin{};
  }
  DraftQuarantinedBootGuardLabState state() const { return state_; }

 private:
  struct Pin {
    std::string operator_session_id{},mission_run_id{},signature{};
    std::uint64_t epoch{0},nonce{0},generation{0},revision{0},last_sequence{0};
  };
  DraftQuarantinedBootGuardLabResult make_result(
      DraftQuarantinedReviewLabState audit, bool can_read=false) const {
    DraftQuarantinedBootGuardLabResult result;
    result.state=state_;
    result.audit=audit;
    result.can_read_preview=can_read;
    return result;
  }
  NativeOfflineQuarantinedPreviewReviewLab review_;
  NativeMissionCommandGateway& gateway_;
  NativeBackendMissionUpload& backend_;
  DraftQuarantinedBootGuardLabState state_{
      DraftQuarantinedBootGuardLabState::new_instance};
  Pin pin_{};
};
} // namespace dyx::rpp
