#pragma once
// DYX Autonomous Phase 2H5J: strictly OFFLINE read-only staged PREVIEW view.
// H5I STAGED + H5H QUARANTINED are mandatory and are never changed here.
// No mission backend / gateway mutations, replay-fence release or DDS/actuation.
// Claimed principal/safety fields are NOT authenticated by this lab; production
// adapters must verify them independently. No method ever authorizes motion.

#include "dyx_rpp_cpp17_core/backend_draft_quarantined_handoff_lab.hpp"
#include <cstdint>
#include <exception>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace dyx::rpp {
enum class DraftQuarantinedReviewLabState {
  denied, backend_active, storage_error, staging_missing,
  quarantine_mismatch, staging_mismatch, history_rejected,
  snapshot_rejected, reviewed_preview_only
};
struct DraftQuarantinedReviewLabRequest {
  DraftReconciliationLabIdentity current_identity{};
  bool explicit_operator_review{false};
};
struct DraftQuarantinedReviewLabResult {
  DraftQuarantinedReviewLabState state{DraftQuarantinedReviewLabState::denied};
  std::string mission_run_id{},signature{};
  std::uint64_t old_epoch{0}, quarantined_epoch{0}, nonce{0}, generation{0};
  std::uint64_t revision{0},last_sequence{0};
  std::optional<PreparedPathSnapshot> read_only_snapshot{};
  std::optional<MissionMode> mission_mode{};
  bool can_post{false},can_unquarantine{false},can_load{false};
  bool can_start_or_resume{false},hardware_command_emitted{false};
};

class NativeOfflineQuarantinedPreviewReviewLab final {
 public:
  explicit NativeOfflineQuarantinedPreviewReviewLab(std::string file)
      : file_(std::move(file)) {}

  DraftQuarantinedReviewLabResult inspect(
      const DraftQuarantinedReviewLabRequest& request,
      const NativeMissionCommandGateway& gateway,
      const NativeBackendMissionUpload& backend) const {
    using namespace draft_journal_lab_detail;
    DraftQuarantinedReviewLabResult out;
    const auto& id=request.current_identity;
    if (!id.authenticated || !id.can_read_mission ||
        !id.fresh_session_evidence || !token(id.operator_session_id) ||
        !serializable(id.session_epoch) || id.session_epoch==0 ||
        !request.explicit_operator_review) return out;
    if (gateway.has_active_mission() || backend.has_draft()) {
      out.state=DraftQuarantinedReviewLabState::backend_active;return out;
    }
    preview_checkpoint_detail::DbHandle db(file_,false);
    if (!db || !exec(db.db,"PRAGMA query_only=ON") ||
        !pragma_check(db.db,"PRAGMA journal_mode","wal") ||
        !pragma_check(db.db,"PRAGMA quick_check(1)","ok") ||
        !journal_schema_ok(db.db)) {
      out.state=DraftQuarantinedReviewLabState::storage_error;return out;
    }
    preview_checkpoint_detail::Tx tx(db.db,false);
    if (!tx.active()) {
      out.state=DraftQuarantinedReviewLabState::storage_error;return out;
    }
    Fence fence;
    if (!read_fence(db.db,fence)) {
      out.state=DraftQuarantinedReviewLabState::quarantine_mismatch;return out;
    }
    Statement b(db.db,"SELECT session_id,old_epoch,new_epoch,nonce,"
      "old_generation,new_generation,mission_run_id,signature,revision,"
      "last_sequence,phase FROM dyx_mission_recovery_epoch_barrier WHERE singleton=1");
    if (!b) {out.state=DraftQuarantinedReviewLabState::storage_error;return out;}
    if (sqlite3_step(b.st)!=SQLITE_ROW) {
      out.state=DraftQuarantinedReviewLabState::quarantine_mismatch;return out;
    }
    for (int c : {1,2,3,4,5,8,9}) {
      if (sqlite3_column_type(b.st,c)!=SQLITE_INTEGER ||
          sqlite3_column_int64(b.st,c)<=0) {
        out.state=DraftQuarantinedReviewLabState::quarantine_mismatch;return out;
      }
    }
    std::string session,run,sig,phase;
    if (!text_col(b.st,0,session,128) || !text_col(b.st,6,run,128) ||
        !text_col(b.st,7,sig,128) || !text_col(b.st,10,phase,24) ||
        session!=id.operator_session_id || session!=fence.session ||
        !valid_native_mission_run_id(run) || !valid_path_signature(sig) ||
        phase!="QUARANTINED" ||
        static_cast<std::uint64_t>(sqlite3_column_int64(b.st,2))!=id.session_epoch ||
        static_cast<std::uint64_t>(sqlite3_column_int64(b.st,2))!=fence.epoch ||
        static_cast<std::uint64_t>(sqlite3_column_int64(b.st,3))!=fence.nonce ||
        static_cast<std::uint64_t>(sqlite3_column_int64(b.st,5))!=fence.generation ||
        sqlite3_column_int64(b.st,2)<=sqlite3_column_int64(b.st,1) ||
        sqlite3_column_int64(b.st,4)==std::numeric_limits<sqlite3_int64>::max() ||
        sqlite3_column_int64(b.st,5)!=sqlite3_column_int64(b.st,4)+1) {
      out.state=DraftQuarantinedReviewLabState::quarantine_mismatch;return out;
    }
    const auto old_epoch=static_cast<std::uint64_t>(sqlite3_column_int64(b.st,1));
    const auto rev=static_cast<std::uint64_t>(sqlite3_column_int64(b.st,8));
    const auto seq=static_cast<std::uint64_t>(sqlite3_column_int64(b.st,9));
    // H5I stage must exist and byte-match the immutable PREVIEW checkpoint.
    Statement s(db.db,"SELECT session_id,old_epoch,new_epoch,nonce,generation,"
      "mission_run_id,signature,revision,last_sequence,snapshot_json,phase "
      "FROM dyx_mission_recovery_staging WHERE singleton=1");
    if (!s) {out.state=DraftQuarantinedReviewLabState::storage_error;return out;}
    if (sqlite3_step(s.st)!=SQLITE_ROW) {
      out.state=DraftQuarantinedReviewLabState::staging_missing;return out;
    }
    for (int c : {1,2,3,4,7,8}) {
      if (sqlite3_column_type(s.st,c)!=SQLITE_INTEGER ||
          sqlite3_column_int64(s.st,c)<=0) {
        out.state=DraftQuarantinedReviewLabState::staging_mismatch;return out;
      }
    }
    std::string ss,sr,sg,json,sp;
    if (!text_col(s.st,0,ss,128) || !text_col(s.st,5,sr,128) ||
        !text_col(s.st,6,sg,128) || !text_col(s.st,9,json,262144) ||
        !text_col(s.st,10,sp,24) || sp!="STAGED" ||
        ss!=session || sr!=run || sg!=sig ||
        static_cast<std::uint64_t>(sqlite3_column_int64(s.st,1))!=old_epoch ||
        static_cast<std::uint64_t>(sqlite3_column_int64(s.st,2))!=fence.epoch ||
        static_cast<std::uint64_t>(sqlite3_column_int64(s.st,3))!=fence.nonce ||
        static_cast<std::uint64_t>(sqlite3_column_int64(s.st,4))!=fence.generation ||
        static_cast<std::uint64_t>(sqlite3_column_int64(s.st,7))!=rev ||
        static_cast<std::uint64_t>(sqlite3_column_int64(s.st,8))!=seq) {
      out.state=DraftQuarantinedReviewLabState::staging_mismatch;return out;
    }
    // Verify exact checkpoint and last COMMITTED PREVIEW bytes in the same
    // SQLite snapshot. The full H5D history is checked before any data return.
    Statement ck(db.db,"SELECT schema_version,session_id,epoch,nonce,"
      "mission_run_id,revision,signature,snapshot_json "
      "FROM dyx_preview_checkpoint WHERE singleton=1");
    if (!ck || sqlite3_step(ck.st)!=SQLITE_ROW ||
        sqlite3_column_type(ck.st,0)!=SQLITE_INTEGER ||
        sqlite3_column_int(ck.st,0)!=1 ||
        sqlite3_column_type(ck.st,2)!=SQLITE_INTEGER ||
        sqlite3_column_type(ck.st,3)!=SQLITE_INTEGER ||
        sqlite3_column_type(ck.st,5)!=SQLITE_INTEGER) {
      out.state=DraftQuarantinedReviewLabState::snapshot_rejected;return out;
    }
    std::string cs,cr,csig,cjson;
    if (!text_col(ck.st,1,cs,128) || !text_col(ck.st,4,cr,128) ||
        !text_col(ck.st,6,csig,128) || !text_col(ck.st,7,cjson,262144) ||
        cs!=session || cr!=run || csig!=sig || cjson!=json ||
        static_cast<std::uint64_t>(sqlite3_column_int64(ck.st,2))!=old_epoch ||
        static_cast<std::uint64_t>(sqlite3_column_int64(ck.st,3))!=fence.nonce ||
        static_cast<std::uint64_t>(sqlite3_column_int64(ck.st,5))!=rev) {
      out.state=DraftQuarantinedReviewLabState::snapshot_rejected;return out;
    }
    preview_checkpoint_detail::Record last;
    if (!preview_checkpoint_detail::latest(db.db,last) ||
        last.nonce!=fence.nonce || last.run!=run ||
        last.status!="COMMITTED" ||
        (last.route!=kBackendMissionUploadPath &&
         last.route!=kBackendMissionUpdatePath) || last.snapshot_json!=json) {
      out.state=DraftQuarantinedReviewLabState::snapshot_rejected;return out;
    }
    Statement seq_st(db.db,"SELECT sequence FROM dyx_mission_mutations "
                           "ORDER BY nonce DESC LIMIT 1");
    if (!seq_st || sqlite3_step(seq_st.st)!=SQLITE_ROW ||
        sqlite3_column_type(seq_st.st,0)!=SQLITE_INTEGER ||
        sqlite3_column_int64(seq_st.st,0)<=0 ||
        static_cast<std::uint64_t>(sqlite3_column_int64(seq_st.st,0))!=seq) {
      out.state=DraftQuarantinedReviewLabState::history_rejected;return out;
    }
    DraftReconciliationLabIdentity historic{true,true,true,session,old_epoch};
    const auto history=NativeOfflineDraftHistoryScreenLab(file_)
                           .inspect_quarantined(historic);
    if (history.state!=DraftHistoryScreenLabState::consistent_preview_history ||
        !history.history_consistency_verified || history.mission_run_id!=run ||
        history.revision!=rev || history.signature!=sig ||
        history.committed_count==0) {
      out.state=DraftQuarantinedReviewLabState::history_rejected;return out;
    }
    draft_audit_lab_detail::StrictSnapshotDecoder::Parsed parsed;
    if (!draft_audit_lab_detail::StrictSnapshotDecoder(json).decode(parsed) ||
        parsed.loaded || parsed.operator_session!=session || parsed.run!=run ||
        parsed.revision!=rev || parsed.snapshot.signature!=sig ||
        parsed.snapshot.marking_points.size()!=history.marking_count ||
        parsed.snapshot.navigation_points.size()!=history.navigation_count) {
      out.state=DraftQuarantinedReviewLabState::snapshot_rejected;return out;
    }
    try {
      const NativeMissionInstallation validated(parsed.snapshot,run,parsed.mode);
      if (validated.marking_goals.size()!=parsed.snapshot.marking_points.size()) {
        out.state=DraftQuarantinedReviewLabState::snapshot_rejected;return out;
      }
    } catch (const std::exception&) {
      out.state=DraftQuarantinedReviewLabState::snapshot_rejected;return out;
    }
    out.state=DraftQuarantinedReviewLabState::reviewed_preview_only;
    out.mission_run_id=run;out.signature=sig;
    out.old_epoch=old_epoch;out.quarantined_epoch=fence.epoch;
    out.nonce=fence.nonce;out.generation=fence.generation;
    out.revision=rev;out.last_sequence=seq;
    out.read_only_snapshot=std::move(parsed.snapshot);
    out.mission_mode=parsed.mode;
    return out;
  }

  BackendMissionUploadResponse handle(
      const BackendMissionUploadRequest& request,
      const DraftQuarantinedReviewLabRequest& review,
      const NativeMissionCommandGateway& gateway,
      const NativeBackendMissionUpload& backend) const {
    BackendMissionUploadResponse reply;
    if (request.path!=kBackendMissionDraftPath) {
      reply.http_status=423;reply.code="RECOVERY_QUARANTINE_COMMANDS_LOCKED";
      return reply;
    }
    if (request.method!="GET") {
      reply.http_status=405;reply.code="METHOD_NOT_ALLOWED";return reply;
    }
    if (!request.principal.authenticated) {
      reply.http_status=401;reply.code="AUTHENTICATION_REQUIRED";return reply;
    }
    if (!request.principal.can_read_mission) {
      reply.http_status=403;reply.code="READ_SCOPE_REQUIRED";return reply;
    }
    if (request.principal.operator_session_id!=review.current_identity.operator_session_id) {
      reply.http_status=403;reply.code="DRAFT_OWNER_REQUIRED";return reply;
    }
    const auto view=inspect(review,gateway,backend);
    if (view.state!=DraftQuarantinedReviewLabState::reviewed_preview_only ||
        !view.read_only_snapshot) {
      reply.http_status=423;reply.code="RECOVERY_PREVIEW_UNAVAILABLE";return reply;
    }
    if (request.mission_run_id!=view.mission_run_id) {
      reply.http_status=403;reply.code="DRAFT_OWNER_REQUIRED";return reply;
    }
    reply.http_status=200;reply.code="QUARANTINED_PREVIEW_READ_ONLY";
    reply.signature=view.signature;reply.revision=view.revision;
    reply.marking_count=view.read_only_snapshot->marking_points.size();
    reply.navigation_count=view.read_only_snapshot->navigation_points.size();
    reply.draft_ready=true;reply.preview_only=true;
    return reply;
  }
 private:
  std::string file_;
};
} // namespace dyx::rpp
