#pragma once
// DYX Autonomous Phase 2H5G: detached, strictly READ-ONLY reconstruction of
// an operator-review PREVIEW after restart. This is an OFFLINE LAB component,
// NOT a production backend restore. It never changes NativeBackendMissionUpload,
// NativeMissionCommandGateway, the durable replay fence, or journal ownership.
// A trusted caller must independently authenticate the operator and obtain
// fresh real PX4 evidence in a later production integration; test booleans
// supplied to H5F are NOT proof of disarm, stationary state or ownership.
// No LOAD / START / RESUME, DDS or actuator action can be issued here.

#include "dyx_rpp_cpp17_core/backend_draft_recovery_preflight_lab.hpp"
#include "dyx_rpp_cpp17_core/mission_runtime_install.hpp"
#include <sqlite3.h>
#include <cstdint>
#include <exception>
#include <optional>
#include <string>
#include <utility>

namespace dyx::rpp {

enum class DraftRecoveredPreviewLabState {
  unrecovered, preflight_rejected, storage_error, replay_metadata_mismatch,
  geometry_rejected, gateway_active, preview_reconstructed, already_reconstructed
};

struct DraftRecoveredPreviewLabResult {
  DraftRecoveredPreviewLabState state{DraftRecoveredPreviewLabState::unrecovered};
  std::string mission_run_id{}, signature{};
  std::uint64_t revision{0}, saved_epoch{0}, proposed_epoch{0},
                last_nonce{0}, last_sequence{0};
  bool can_restore_backend{false}, can_load{false}, can_start_or_resume{false};
  bool hardware_command_emitted{false};
};

class NativeOfflineRecoveredDraftPreviewLab final {
 public:
  explicit NativeOfflineRecoveredDraftPreviewLab(std::string existing_db)
      : file_(std::move(existing_db)) {}

  // Build an isolated, memory-only READ model, never the command backend.
  // Immutable once created: subsequent attempts cannot replace its contents.
  DraftRecoveredPreviewLabResult reconstruct_for_review(
      const DraftRecoveryPreflightLabRequest& request,
      const NativeMissionCommandGateway& gateway) {
    using namespace draft_journal_lab_detail;
    using namespace preview_checkpoint_detail;
    DraftRecoveredPreviewLabResult result;
    if (view_) {
      result.state=DraftRecoveredPreviewLabState::already_reconstructed;
      return result;
    }
    if (gateway.has_active_mission()) {
      result.state=DraftRecoveredPreviewLabState::gateway_active;
      return result;
    }
    DbHandle db(file_,false);
    if (!db || !exec(db.db,"PRAGMA query_only=ON") ||
        !pragma_check(db.db,"PRAGMA journal_mode","wal") ||
        !pragma_check(db.db,"PRAGMA quick_check(1)","ok")) {
      result.state=DraftRecoveredPreviewLabState::storage_error;
      return result;
    }
    Tx transaction(db.db,false);
    if (!transaction.active()) {
      result.state=DraftRecoveredPreviewLabState::storage_error;
      return result;
    }
    // Establish an unchanging DB snapshot before H5F opens its own reader.
    Fence fence;
    if (!read_fence(db.db,fence)) {
      result.state=DraftRecoveredPreviewLabState::replay_metadata_mismatch;
      return result;
    }
    const auto preflight=NativeOfflineDraftRecoveryPreflightLab(file_).inspect(request);
    if (preflight.state!=DraftRecoveryPreflightLabState::reviewed_preview_only ||
        !preflight.reviewed_snapshot) {
      result.state=DraftRecoveredPreviewLabState::preflight_rejected;
      return result;
    }
    if (fence.session!=preflight.operator_session_id ||
        fence.epoch!=preflight.old_epoch || fence.nonce!=preflight.last_nonce ||
        !valid_native_mission_run_id(preflight.mission_run_id) ||
        !valid_path_signature(preflight.signature) ||
        preflight.revision==0 || preflight.last_sequence==0) {
      result.state=DraftRecoveredPreviewLabState::replay_metadata_mismatch;
      return result;
    }
    Record last;
    draft_audit_lab_detail::StrictSnapshotDecoder::Parsed parsed;
    if (!latest(db.db,last) || last.status!="COMMITTED" ||
        (last.route!=kBackendMissionUploadPath &&
         last.route!=kBackendMissionUpdatePath) ||
        last.nonce!=preflight.last_nonce ||
        last.run!=preflight.mission_run_id ||
        !draft_audit_lab_detail::StrictSnapshotDecoder(last.snapshot_json).decode(parsed) ||
        parsed.loaded || parsed.run!=preflight.mission_run_id ||
        parsed.operator_session!=preflight.operator_session_id ||
        parsed.revision!=preflight.revision ||
        parsed.snapshot.signature!=preflight.signature ||
        parsed.snapshot.marking_points.size()!=preflight.reviewed_snapshot->marking_points.size() ||
        parsed.snapshot.navigation_points.size()!=preflight.reviewed_snapshot->navigation_points.size()) {
      result.state=DraftRecoveredPreviewLabState::replay_metadata_mismatch;
      return result;
    }
    // Compare the exact canonical complete snapshot bytes against H5A's
    // serializer. Signature checks consistency, NOT database authentication.
    BackendMissionUploadRequest binding;
    binding.path=last.route;
    binding.mission_run_id=last.run;
    binding.principal.operator_session_id=parsed.operator_session;
    BackendMissionUploadResponse response;
    response.http_status=200;
    response.revision=parsed.revision;
    response.signature=parsed.snapshot.signature;
    response.marking_count=parsed.snapshot.marking_points.size();
    response.navigation_count=parsed.snapshot.navigation_points.size();
    response.draft_ready=true;
    std::string canonical;
    if (!journaled_upload_lab_detail::json_snapshot(response,binding,
            &parsed.snapshot,parsed.mode,canonical) ||
        canonical!=last.snapshot_json) {
      result.state=DraftRecoveredPreviewLabState::replay_metadata_mismatch;
      return result;
    }
    try {
      // Strict geometry/goal validation through the genuine non-actuating
      // installation constructor, on a discarded LOCAL temporary object.
      // Never call gateway.load(), sequencer.start(), or backend.handle().
      const NativeMissionInstallation validated(
          parsed.snapshot, parsed.run, parsed.mode);
      if (validated.marking_goals.size()!=parsed.snapshot.marking_points.size()) {
        result.state=DraftRecoveredPreviewLabState::geometry_rejected;
        return result;
      }
    } catch (const std::exception&) {
      result.state=DraftRecoveredPreviewLabState::geometry_rejected;
      return result;
    }
    View staged;
    staged.run=std::move(parsed.run);
    staged.operator_session=std::move(parsed.operator_session);
    staged.snapshot=std::move(parsed.snapshot);
    staged.mode=parsed.mode;
    staged.revision=preflight.revision;
    staged.epoch=preflight.old_epoch;
    staged.proposed_epoch=preflight.proposed_new_epoch;
    staged.nonce=preflight.last_nonce;
    staged.sequence=preflight.last_sequence;
    // All of the above data comes from a pinned read transaction. Moving the
    // detached snapshot to the facade cannot grant command-side authority.
    view_=std::move(staged);
    result.state=DraftRecoveredPreviewLabState::preview_reconstructed;
    result.mission_run_id=view_->run;
    result.signature=view_->snapshot.signature;
    result.revision=view_->revision;
    result.saved_epoch=view_->epoch;
    result.proposed_epoch=view_->proposed_epoch;
    result.last_nonce=view_->nonce;
    result.last_sequence=view_->sequence;
    return result;
  }

  // Deliberately returns HTTP-style 423 for every command/mutation route,
  // regardless of leases, sequence numbers or forged PX4 claims.
  BackendMissionUploadResponse handle(const BackendMissionUploadRequest& req) const {
    BackendMissionUploadResponse response;
    const bool draft=req.path==kBackendMissionDraftPath;
    if (!draft) {
      response.http_status=423;
      response.code="RECOVERED_PREVIEW_COMMANDS_LOCKED";
      return response;
    }
    if (req.method!="GET") {
      response.http_status=405;
      response.code="METHOD_NOT_ALLOWED";
      return response;
    }
    if (!req.principal.authenticated) {
      response.http_status=401;
      response.code="AUTHENTICATION_REQUIRED";
      return response;
    }
    if (!req.principal.can_read_mission) {
      response.http_status=403;
      response.code="READ_SCOPE_REQUIRED";
      return response;
    }
    if (!view_) {
      response.http_status=404;
      response.code="NO_RECOVERED_PREVIEW";
      return response;
    }
    if (req.principal.operator_session_id!=view_->operator_session ||
        req.mission_run_id!=view_->run) {
      response.http_status=403;
      response.code="DRAFT_OWNER_REQUIRED";
      return response;
    }
    response.http_status=200;
    response.code="RECOVERED_PREVIEW_READ_ONLY";
    response.revision=view_->revision;
    response.signature=view_->snapshot.signature;
    response.marking_count=view_->snapshot.marking_points.size();
    response.navigation_count=view_->snapshot.navigation_points.size();
    response.draft_ready=true;
    response.loaded=false;
    response.preview_only=true;
    return response;
  }

  // Only an authorized, bound GET can access the reconstructed geometry.
  // Return by VALUE: the caller cannot mutate the facade's saved copy.
  std::optional<PreparedPathSnapshot> review_snapshot(
      const BackendMissionUploadRequest& req) const {
    if (handle(req).http_status!=200 || !view_)return std::nullopt;
    return view_->snapshot;
  }
  std::optional<MissionMode> review_mode(
      const BackendMissionUploadRequest& req) const {
    if (handle(req).http_status!=200 || !view_)return std::nullopt;
    return view_->mode;
  }
  bool has_recovered_preview() const {return view_.has_value();}
 private:
  struct View {
    std::string run{},operator_session{};
    PreparedPathSnapshot snapshot{};
    MissionMode mode{MissionMode::automatic};
    std::uint64_t revision{0},epoch{0},proposed_epoch{0},nonce{0},sequence{0};
  };
  std::string file_;
  std::optional<View> view_{};
};

}  // namespace dyx::rpp
