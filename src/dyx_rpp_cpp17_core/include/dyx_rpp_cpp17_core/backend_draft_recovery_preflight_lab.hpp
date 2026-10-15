#pragma once
// DYX Autonomous Phase 2H5F: OFFLINE / READ-ONLY PREVIEW recovery preflight.
// An operator-reviewed *rehearsal record*, NOT a backend restore capability.
// This class never mutates a backend, gateway, replay fence or SQLite database;
// never LOADs, STARTs, RESUMEs or emits motor/spray/laser/DDS commands.
// The boolean claims provided here are NOT independent authentication or PX4
// attestation. Until backed by a privileged live service they are only test
// fixtures; they do not grant any authority. All result action flags stay false.

#include "dyx_rpp_cpp17_core/backend_draft_preview_checkpoint_lab.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace dyx::rpp {
enum class DraftRecoveryPreflightLabState {
  access_denied, operator_review_required, new_epoch_required,
  safety_evidence_required, checkpoint_unavailable,
  inconsistent_replay_metadata, reviewed_preview_only
};
struct DraftRecoveryPreflightLabRequest {
  DraftReconciliationLabIdentity saved_identity{};
  std::uint64_t proposed_new_epoch{0};
  bool explicit_operator_review{false};
  bool independently_disarmed{false};
  bool independently_stationary{false};
  bool independently_no_active_mission{false};
};
struct DraftRecoveryPreflightLabResult {
  DraftRecoveryPreflightLabState state{DraftRecoveryPreflightLabState::checkpoint_unavailable};
  std::string mission_run_id{}, signature{}, operator_session_id{};
  std::uint64_t old_epoch{0}, proposed_new_epoch{0}, last_nonce{0},
                last_sequence{0}, revision{0};
  double last_received_sec{0.0};
  // For offline review and comparison ONLY, never for direct LOAD/START.
  std::optional<PreparedPathSnapshot> reviewed_snapshot{};
  bool can_rehydrate_backend{false};
  bool can_load{false};
  bool can_start_or_resume{false};
  bool hardware_command_emitted{false};
};

class NativeOfflineDraftRecoveryPreflightLab final {
 public:
  explicit NativeOfflineDraftRecoveryPreflightLab(std::string existing_file)
      : file_(std::move(existing_file)) {}

  DraftRecoveryPreflightLabResult inspect(
      const DraftRecoveryPreflightLabRequest& req) const {
    using namespace draft_journal_lab_detail;
    using namespace preview_checkpoint_detail;
    DraftRecoveryPreflightLabResult result;
    const auto& id=req.saved_identity;
    if (!id.authenticated || !id.can_read_mission ||
        !id.fresh_session_evidence || !token(id.operator_session_id) ||
        !serializable(id.session_epoch) || id.session_epoch==0) {
      result.state=DraftRecoveryPreflightLabState::access_denied;
      return result;
    }
    if (!req.explicit_operator_review) {
      result.state=DraftRecoveryPreflightLabState::operator_review_required;
      return result;
    }
    if (!serializable(req.proposed_new_epoch) ||
        req.proposed_new_epoch<=id.session_epoch) {
      result.state=DraftRecoveryPreflightLabState::new_epoch_required;
      return result;
    }
    if (!req.independently_disarmed || !req.independently_stationary ||
        !req.independently_no_active_mission) {
      result.state=DraftRecoveryPreflightLabState::safety_evidence_required;
      return result;
    }
    DbHandle db(file_,false);
    if (!db || !exec(db.db,"PRAGMA query_only=ON") ||
        !pragma_check(db.db,"PRAGMA journal_mode","wal") ||
        !pragma_check(db.db,"PRAGMA quick_check(1)","ok"))return result;
    Tx tx(db.db,false);
    if (!tx.active())return result;
    // Pin the SQLite snapshot BEFORE inspecting another connection. Any
    // journal mutation between the two readers creates a nonce mismatch.
    Fence fence;
    if (!read_fence(db.db,fence)) {
      result.state=DraftRecoveryPreflightLabState::inconsistent_replay_metadata;
      return result;
    }
    if (fence.session!=id.operator_session_id || fence.epoch!=id.session_epoch) {
      result.state=DraftRecoveryPreflightLabState::access_denied;
      return result;
    }
    if (fence.nonce==0 || !std::isfinite(fence.sec) || fence.sec<0.0) {
      result.state=DraftRecoveryPreflightLabState::inconsistent_replay_metadata;
      return result;
    }
    const auto preview=NativeOfflineDraftPreviewCheckpointLab(file_).inspect_preview(id);
    if (preview.state!=DraftPreviewCheckpointLabState::verified_preview ||
        !preview.review_snapshot) {
      result.state=DraftRecoveryPreflightLabState::checkpoint_unavailable;
      return result;
    }
    if (preview.ingress_nonce!=fence.nonce || preview.revision==0 ||
        !valid_native_mission_run_id(preview.mission_run_id) ||
        !valid_path_signature(preview.signature) ||
        preview.signature!=preview.review_snapshot->signature) {
      result.state=DraftRecoveryPreflightLabState::inconsistent_replay_metadata;
      return result;
    }
    Statement st(db.db,"SELECT nonce,session_id,epoch,mission_run_id,sequence,"
                       "route,status,stage FROM dyx_mission_mutations "
                       "ORDER BY nonce DESC LIMIT 1");
    if (!st || sqlite3_step(st.st)!=SQLITE_ROW ||
        sqlite3_column_type(st.st,0)!=SQLITE_INTEGER ||
        sqlite3_column_type(st.st,2)!=SQLITE_INTEGER ||
        sqlite3_column_type(st.st,4)!=SQLITE_INTEGER) {
      result.state=DraftRecoveryPreflightLabState::inconsistent_replay_metadata;
      return result;
    }
    const auto nonce=sqlite3_column_int64(st.st,0);
    const auto epoch=sqlite3_column_int64(st.st,2);
    const auto sequence=sqlite3_column_int64(st.st,4);
    std::string session,run,route_name,status,stage;
    if (nonce<=0 || sequence<=0 || epoch<=0 ||
        static_cast<std::uint64_t>(nonce)!=fence.nonce ||
        static_cast<std::uint64_t>(epoch)!=id.session_epoch ||
        !text_col(st.st,1,session,128) || !text_col(st.st,3,run,128) ||
        !text_col(st.st,5,route_name,64) || !text_col(st.st,6,status,16) ||
        !text_col(st.st,7,stage,16) ||
        session!=id.operator_session_id || run!=preview.mission_run_id ||
        status!="COMMITTED" || stage!="PREVIEW" ||
        (route_name!=kBackendMissionUploadPath &&
         route_name!=kBackendMissionUpdatePath)) {
      result.state=DraftRecoveryPreflightLabState::inconsistent_replay_metadata;
      return result;
    }
    // Do not advance the saved epoch or unquarantine a writer. The proposed
    // epoch is informational until a future atomic epoch-transition protocol.
    result.state=DraftRecoveryPreflightLabState::reviewed_preview_only;
    result.operator_session_id=session;
    result.mission_run_id=run;
    result.signature=preview.signature;
    result.revision=preview.revision;
    result.old_epoch=id.session_epoch;
    result.proposed_new_epoch=req.proposed_new_epoch;
    result.last_nonce=fence.nonce;
    result.last_sequence=static_cast<std::uint64_t>(sequence);
    result.last_received_sec=fence.sec;
    result.reviewed_snapshot=std::move(preview.review_snapshot);
    return result;
  }
 private:
  std::string file_;
};
} // namespace dyx::rpp
