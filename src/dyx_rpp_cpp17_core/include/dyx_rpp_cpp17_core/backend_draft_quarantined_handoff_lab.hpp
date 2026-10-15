#pragma once
// DYX Autonomous H5I: offline, PREVIEW-only, immutable handoff staging.
// Requires the H5H durable write barrier to EXIST and remain QUARANTINED.
// This is not a backend rehydration API. Never releases the journal,
// touches a mission gateway/backend, sends DDS or commands any actuator.
// All request booleans are laboratory claims, NOT verified real PX4 evidence.

#include "dyx_rpp_cpp17_core/backend_draft_epoch_quarantine_lab.hpp"
#include <cstdint>
#include <exception>
#include <optional>
#include <string>
#include <utility>

namespace dyx::rpp {
enum class DraftQuarantinedHandoffLabState {
  denied, backend_active, storage_error, not_quarantined,
  inconsistent_barrier, history_rejected, snapshot_rejected,
  staging_conflict, write_failure, staged_preview, already_staged
};
struct DraftQuarantinedHandoffLabRequest {
  DraftReconciliationLabIdentity current_identity{};
  bool explicit_operator_review{false};
  bool independently_disarmed{false};
  bool independently_stationary{false};
  bool independently_no_active_mission{false};
};
struct DraftQuarantinedHandoffLabResult {
  DraftQuarantinedHandoffLabState state{DraftQuarantinedHandoffLabState::denied};
  std::string mission_run_id{},signature{};
  std::uint64_t old_epoch{0},quarantined_epoch{0},nonce{0},generation{0};
  std::uint64_t revision{0},last_sequence{0};
  std::optional<PreparedPathSnapshot> read_only_preview{};
  bool can_unquarantine{false},can_load{false},can_start_or_resume{false};
  bool can_post{false},hardware_command_emitted{false};
};

class NativeOfflineQuarantinedPreviewHandoffLab final {
 public:
  explicit NativeOfflineQuarantinedPreviewHandoffLab(std::string file)
      : file_(std::move(file)) {}

  DraftQuarantinedHandoffLabResult stage_for_review(
      const DraftQuarantinedHandoffLabRequest& req,
      const NativeMissionCommandGateway& gateway,
      const NativeBackendMissionUpload& backend) const {
    using namespace draft_journal_lab_detail;
    DraftQuarantinedHandoffLabResult out;
    const auto& id=req.current_identity;
    if (!id.authenticated || !id.can_read_mission ||
        !id.fresh_session_evidence || !token(id.operator_session_id) ||
        !serializable(id.session_epoch) || id.session_epoch==0 ||
        !req.explicit_operator_review || !req.independently_disarmed ||
        !req.independently_stationary || !req.independently_no_active_mission)
      return out;
    if (gateway.has_active_mission() || backend.has_draft()) {
      out.state=DraftQuarantinedHandoffLabState::backend_active;
      return out;
    }
    sqlite3* db=nullptr;
    try {db=open_existing(file_);} catch(const std::exception&) {
      out.state=DraftQuarantinedHandoffLabState::storage_error;
      return out;
    }
    struct Close {sqlite3* db; ~Close(){sqlite3_close_v2(db);} } close{db};
    if (!policy(db) || !journal_schema_ok(db) || !exec(db,"BEGIN IMMEDIATE")) {
      out.state=DraftQuarantinedHandoffLabState::storage_error;
      return out;
    }
    struct Rollback {sqlite3* db; bool committed{false};
      ~Rollback(){if(!committed)exec(db,"ROLLBACK");} } transaction{db};
    Fence fence;
    if (!read_fence(db,fence)) {
      out.state=DraftQuarantinedHandoffLabState::inconsistent_barrier;
      return out;
    }
    Statement b(db,"SELECT session_id,old_epoch,new_epoch,nonce,"
      "old_generation,new_generation,mission_run_id,signature,revision,"
      "last_sequence,phase FROM dyx_mission_recovery_epoch_barrier WHERE singleton=1");
    if (!b) {out.state=DraftQuarantinedHandoffLabState::storage_error;return out;}
    const int step=sqlite3_step(b.st);
    if (step==SQLITE_DONE) {
      out.state=DraftQuarantinedHandoffLabState::not_quarantined;
      return out;
    }
    if (step!=SQLITE_ROW) {
      out.state=DraftQuarantinedHandoffLabState::inconsistent_barrier;
      return out;
    }
    for (int col : {1,2,3,4,5,8,9}) {
      if (sqlite3_column_type(b.st,col)!=SQLITE_INTEGER ||
          sqlite3_column_int64(b.st,col)<=0) {
        out.state=DraftQuarantinedHandoffLabState::inconsistent_barrier;return out;
      }
    }
    std::string session,run,signature,phase;
    if (!text_col(b.st,0,session,128) || !text_col(b.st,6,run,128) ||
        !text_col(b.st,7,signature,128) || !text_col(b.st,10,phase,24) ||
        !token(session) || !valid_native_mission_run_id(run) ||
        !valid_path_signature(signature) || phase!="QUARANTINED" ||
        session!=id.operator_session_id || session!=fence.session ||
        static_cast<std::uint64_t>(sqlite3_column_int64(b.st,2))!=id.session_epoch ||
        static_cast<std::uint64_t>(sqlite3_column_int64(b.st,2))!=fence.epoch ||
        static_cast<std::uint64_t>(sqlite3_column_int64(b.st,3))!=fence.nonce ||
        static_cast<std::uint64_t>(sqlite3_column_int64(b.st,5))!=fence.generation ||
        sqlite3_column_int64(b.st,2)<=sqlite3_column_int64(b.st,1) ||
        sqlite3_column_int64(b.st,5)!=sqlite3_column_int64(b.st,4)+1) {
      out.state=DraftQuarantinedHandoffLabState::inconsistent_barrier;
      return out;
    }
    const auto old_epoch=static_cast<std::uint64_t>(sqlite3_column_int64(b.st,1));
    const auto revision=static_cast<std::uint64_t>(sqlite3_column_int64(b.st,8));
    const auto sequence=static_cast<std::uint64_t>(sqlite3_column_int64(b.st,9));
    // Full H5D canonical mutation-history scan, but bound to the H5H rotated
    // fence/barrier rather than mistakenly treating the new epoch as old history.
    DraftReconciliationLabIdentity historic={true,true,true,session,old_epoch};
    const auto screen=NativeOfflineDraftHistoryScreenLab(file_)
                            .inspect_quarantined(historic);
    if (screen.state!=DraftHistoryScreenLabState::consistent_preview_history ||
        !screen.history_consistency_verified || screen.mission_run_id!=run ||
        screen.revision!=revision || screen.signature!=signature ||
        screen.committed_count==0) {
      out.state=DraftQuarantinedHandoffLabState::history_rejected;
      return out;
    }
    Statement st(db,"SELECT schema_version,session_id,epoch,nonce,"
      "mission_run_id,revision,signature,snapshot_json "
      "FROM dyx_preview_checkpoint WHERE singleton=1");
    if (!st || sqlite3_step(st.st)!=SQLITE_ROW ||
        sqlite3_column_type(st.st,0)!=SQLITE_INTEGER ||
        sqlite3_column_int(st.st,0)!=1 ||
        sqlite3_column_type(st.st,2)!=SQLITE_INTEGER ||
        sqlite3_column_type(st.st,3)!=SQLITE_INTEGER ||
        sqlite3_column_type(st.st,5)!=SQLITE_INTEGER) {
      out.state=DraftQuarantinedHandoffLabState::snapshot_rejected;
      return out;
    }
    std::string ck_session,ck_run,ck_signature,canonical;
    if (!text_col(st.st,1,ck_session,128) ||
        !text_col(st.st,4,ck_run,128) ||
        !text_col(st.st,6,ck_signature,128) ||
        !text_col(st.st,7,canonical,262144) ||
        ck_session!=session || ck_run!=run || ck_signature!=signature ||
        sqlite3_column_int64(st.st,2)!=static_cast<sqlite3_int64>(old_epoch) ||
        sqlite3_column_int64(st.st,3)!=static_cast<sqlite3_int64>(fence.nonce) ||
        sqlite3_column_int64(st.st,5)!=static_cast<sqlite3_int64>(revision)) {
      out.state=DraftQuarantinedHandoffLabState::snapshot_rejected;
      return out;
    }
    Statement last_sequence(db,"SELECT sequence FROM dyx_mission_mutations "
                               "ORDER BY nonce DESC LIMIT 1");
    if (!last_sequence || sqlite3_step(last_sequence.st)!=SQLITE_ROW ||
        sqlite3_column_type(last_sequence.st,0)!=SQLITE_INTEGER ||
        sqlite3_column_int64(last_sequence.st,0)<=0 ||
        static_cast<std::uint64_t>(sqlite3_column_int64(last_sequence.st,0))!=sequence) {
      out.state=DraftQuarantinedHandoffLabState::history_rejected;
      return out;
    }
    preview_checkpoint_detail::Record last;
    if (!preview_checkpoint_detail::latest(db,last) || last.nonce!=fence.nonce ||
        last.status!="COMMITTED" || last.run!=run ||
        (last.route!=kBackendMissionUploadPath && last.route!=kBackendMissionUpdatePath) ||
        last.snapshot_json!=canonical) {
      out.state=DraftQuarantinedHandoffLabState::snapshot_rejected;
      return out;
    }
    draft_audit_lab_detail::StrictSnapshotDecoder::Parsed parsed;
    if (!draft_audit_lab_detail::StrictSnapshotDecoder(canonical).decode(parsed) ||
        parsed.loaded || parsed.operator_session!=session || parsed.run!=run ||
        parsed.revision!=revision || parsed.snapshot.signature!=signature ||
        parsed.snapshot.marking_points.size()!=screen.marking_count ||
        parsed.snapshot.navigation_points.size()!=screen.navigation_count) {
      out.state=DraftQuarantinedHandoffLabState::snapshot_rejected;
      return out;
    }
    try {
      const NativeMissionInstallation validated(parsed.snapshot,run,parsed.mode);
      if (validated.marking_goals.size()!=parsed.snapshot.marking_points.size()) {
        out.state=DraftQuarantinedHandoffLabState::snapshot_rejected;return out;
      }
    } catch (const std::exception&) {
      out.state=DraftQuarantinedHandoffLabState::snapshot_rejected;return out;
    }
    Statement put(db,"INSERT OR IGNORE INTO dyx_mission_recovery_staging("
      "singleton,session_id,old_epoch,new_epoch,nonce,generation,"
      "mission_run_id,signature,revision,last_sequence,snapshot_json,phase)"
      "VALUES(1,?,?,?,?,?,?,?,?,?,?,'STAGED')");
    if (!put || !bind_text(put.st,1,session) || !bind_integer(put.st,2,old_epoch) ||
        !bind_integer(put.st,3,fence.epoch) || !bind_integer(put.st,4,fence.nonce) ||
        !bind_integer(put.st,5,fence.generation) || !bind_text(put.st,6,run) ||
        !bind_text(put.st,7,signature) || !bind_integer(put.st,8,revision) ||
        !bind_integer(put.st,9,sequence) || !bind_text(put.st,10,canonical) ||
        sqlite3_step(put.st)!=SQLITE_DONE) {
      out.state=DraftQuarantinedHandoffLabState::write_failure;
      return out;
    }
    const bool inserted=sqlite3_changes(db)==1;
    Statement check(db,"SELECT session_id,old_epoch,new_epoch,nonce,generation,"
      "mission_run_id,signature,revision,last_sequence,snapshot_json,phase "
      "FROM dyx_mission_recovery_staging WHERE singleton=1");
    if (!check || sqlite3_step(check.st)!=SQLITE_ROW) {
      out.state=DraftQuarantinedHandoffLabState::staging_conflict;return out;
    }
    std::string saved_session,saved_run,saved_sig,saved_json,saved_phase;
    for (int col : {1,2,3,4,7,8}) {
      if (sqlite3_column_type(check.st,col)!=SQLITE_INTEGER) {
        out.state=DraftQuarantinedHandoffLabState::staging_conflict;return out;
      }
    }
    if (!text_col(check.st,0,saved_session,128) ||
        !text_col(check.st,5,saved_run,128) ||
        !text_col(check.st,6,saved_sig,128) ||
        !text_col(check.st,9,saved_json,262144) ||
        !text_col(check.st,10,saved_phase,24) ||
        saved_session!=session || saved_run!=run || saved_sig!=signature ||
        saved_json!=canonical || saved_phase!="STAGED" ||
        sqlite3_column_int64(check.st,1)!=static_cast<sqlite3_int64>(old_epoch) ||
        sqlite3_column_int64(check.st,2)!=static_cast<sqlite3_int64>(fence.epoch) ||
        sqlite3_column_int64(check.st,3)!=static_cast<sqlite3_int64>(fence.nonce) ||
        sqlite3_column_int64(check.st,4)!=static_cast<sqlite3_int64>(fence.generation) ||
        sqlite3_column_int64(check.st,7)!=static_cast<sqlite3_int64>(revision) ||
        sqlite3_column_int64(check.st,8)!=static_cast<sqlite3_int64>(sequence)) {
      out.state=DraftQuarantinedHandoffLabState::staging_conflict;return out;
    }
    if (!exec(db,"COMMIT")) {
      out.state=DraftQuarantinedHandoffLabState::write_failure;return out;
    }
    transaction.committed=true;
    out.state=inserted?DraftQuarantinedHandoffLabState::staged_preview:
                       DraftQuarantinedHandoffLabState::already_staged;
    out.mission_run_id=run;out.signature=signature;
    out.old_epoch=old_epoch;out.quarantined_epoch=fence.epoch;
    out.nonce=fence.nonce;out.generation=fence.generation;
    out.revision=revision;out.last_sequence=sequence;
    out.read_only_preview=std::move(parsed.snapshot);
    return out;
  }
 private:
  std::string file_;
};
} // namespace dyx::rpp
