#pragma once
// DYX Autonomous H5H — OFFLINE SQLite epoch rotation into a durable
// QUARANTINED state. This is NOT backend rehydration or an enable-control API.
// The epoch bump, write barrier, and replay-fence generation commit atomically.
// Every subsequent H4Z journal mutation is refused while the barrier exists.
// The barrier has deliberately NO release/undo or automatic recovery method.
// Input booleans are laboratory claims, NOT real PX4/operator attestation.

#include "dyx_rpp_cpp17_core/backend_draft_recovered_preview_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_sqlite_mutation_journal_lab.hpp"
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace dyx::rpp {

enum class DraftEpochQuarantineLabState {
  invalid_request, active_backend_or_gateway, storage_error,
  already_quarantined, preflight_rejected, preview_rejected,
  inconsistent_fence, persistence_fault, quarantine_committed
};

struct DraftEpochQuarantineLabResult {
  DraftEpochQuarantineLabState state{DraftEpochQuarantineLabState::invalid_request};
  std::uint64_t saved_epoch{0}, quarantined_epoch{0};
  std::uint64_t last_nonce{0}, last_sequence{0}, fence_generation{0};
  std::string mission_run_id{}, signature{};
  bool can_post{false}, can_load{false}, can_start_or_resume{false};
  bool hardware_command_emitted{false};
};

class NativeOfflineDraftEpochQuarantineLab final {
 public:
  explicit NativeOfflineDraftEpochQuarantineLab(std::string existing_db)
      : file_(std::move(existing_db)) {}

  // Mutates ONLY an offline SQLite file after full H5F/H5G verification.
  // Must never be wired to an unauthenticated API endpoint. A production
  // design needs independent live safety evidence and an external writer lock.
  DraftEpochQuarantineLabResult quarantine(
      const DraftRecoveryPreflightLabRequest& request,
      const NativeMissionCommandGateway& gateway,
      const NativeBackendMissionUpload& backend) const {
    using namespace draft_journal_lab_detail;
    DraftEpochQuarantineLabResult out;
    const auto& id=request.saved_identity;
    if (!id.authenticated || !id.can_read_mission || !id.fresh_session_evidence ||
        !token(id.operator_session_id) || !serializable(id.session_epoch) ||
        id.session_epoch==0 || !serializable(request.proposed_new_epoch) ||
        request.proposed_new_epoch<=id.session_epoch ||
        !request.explicit_operator_review || !request.independently_disarmed ||
        !request.independently_stationary ||
        !request.independently_no_active_mission) return out;
    if (gateway.has_active_mission() || backend.has_draft()) {
      out.state=DraftEpochQuarantineLabState::active_backend_or_gateway;
      return out;
    }
    sqlite3* db=nullptr;
    try { db=open_existing(file_); }
    catch (const std::exception&) {
      out.state=DraftEpochQuarantineLabState::storage_error;
      return out;
    }
    struct Close { sqlite3* db; ~Close() {if(db) sqlite3_close_v2(db);} } close{db};
    if (!policy(db) || !journal_schema_ok(db)) {
      out.state=DraftEpochQuarantineLabState::storage_error;
      return out;
    }
    // BEGIN IMMEDIATE prevents an H4Z writer from mutating the journal while
    // H5F and H5G independently inspect the same committed PREVIEW.
    if (!exec(db,"BEGIN IMMEDIATE")) {
      out.state=DraftEpochQuarantineLabState::storage_error;
      return out;
    }
    struct Rollback { sqlite3* db; bool committed{false};
      ~Rollback(){if(!committed) exec(db,"ROLLBACK");} } tx{db};
    Statement barrier(db,"SELECT 1 FROM dyx_mission_recovery_epoch_barrier LIMIT 1");
    if (!barrier) {
      out.state=DraftEpochQuarantineLabState::storage_error;
      return out;
    }
    const int b=sqlite3_step(barrier.st);
    if (b==SQLITE_ROW) {
      out.state=DraftEpochQuarantineLabState::already_quarantined;
      return out;
    }
    if (b!=SQLITE_DONE) {
      out.state=DraftEpochQuarantineLabState::storage_error;
      return out;
    }
    Fence fence;
    if (!read_fence(db,fence) || fence.session!=id.operator_session_id ||
        fence.epoch!=id.session_epoch || fence.nonce==0 ||
        fence.generation>=static_cast<std::uint64_t>(std::numeric_limits<sqlite3_int64>::max())) {
      out.state=DraftEpochQuarantineLabState::inconsistent_fence;
      return out;
    }
    const auto preflight=NativeOfflineDraftRecoveryPreflightLab(file_).inspect(request);
    if (preflight.state!=DraftRecoveryPreflightLabState::reviewed_preview_only ||
        !preflight.reviewed_snapshot) {
      out.state=DraftEpochQuarantineLabState::preflight_rejected;
      return out;
    }
    NativeOfflineRecoveredDraftPreviewLab preview(file_);
    const auto verified=preview.reconstruct_for_review(request,gateway);
    if (verified.state!=DraftRecoveredPreviewLabState::preview_reconstructed) {
      out.state=DraftEpochQuarantineLabState::preview_rejected;
      return out;
    }
    if (preflight.old_epoch!=fence.epoch || preflight.last_nonce!=fence.nonce ||
        preflight.operator_session_id!=fence.session ||
        preflight.proposed_new_epoch!=request.proposed_new_epoch ||
        preflight.mission_run_id!=verified.mission_run_id ||
        preflight.signature!=verified.signature ||
        preflight.revision!=verified.revision ||
        preflight.last_sequence!=verified.last_sequence ||
        preflight.last_sequence==0 || !valid_native_mission_run_id(verified.mission_run_id) ||
        !valid_path_signature(verified.signature)) {
      out.state=DraftEpochQuarantineLabState::inconsistent_fence;
      return out;
    }
    // Write a permanent quarantine barrier first, then rotate the fence.
    // Both operations must commit or roll back as a single SQLite transaction.
    Statement insert(db,"INSERT INTO dyx_mission_recovery_epoch_barrier("
                       "singleton,session_id,old_epoch,new_epoch,nonce,"
                       "old_generation,new_generation,mission_run_id,"
                       "signature,revision,last_sequence,phase)"
                       " VALUES(1,?,?,?,?,?,?,?,?,?,?,'QUARANTINED')");
    bool ok=insert && bind_text(insert.st,1,fence.session) &&
        bind_integer(insert.st,2,fence.epoch) &&
        bind_integer(insert.st,3,request.proposed_new_epoch) &&
        bind_integer(insert.st,4,fence.nonce) &&
        bind_integer(insert.st,5,fence.generation) &&
        bind_integer(insert.st,6,fence.generation+1) &&
        bind_text(insert.st,7,verified.mission_run_id) &&
        bind_text(insert.st,8,verified.signature) &&
        bind_integer(insert.st,9,verified.revision) &&
        bind_integer(insert.st,10,verified.last_sequence) &&
        sqlite3_step(insert.st)==SQLITE_DONE && sqlite3_changes(db)==1;
    if (ok) {
      Statement bump(db,"UPDATE replay_fence SET epoch=?,generation=? "
                        "WHERE singleton=1 AND session_id=? AND epoch=? AND "
                        "nonce=? AND generation=?");
      ok=bump && bind_integer(bump.st,1,request.proposed_new_epoch) &&
          bind_integer(bump.st,2,fence.generation+1) &&
          bind_text(bump.st,3,fence.session) &&
          bind_integer(bump.st,4,fence.epoch) &&
          bind_integer(bump.st,5,fence.nonce) &&
          bind_integer(bump.st,6,fence.generation) &&
          sqlite3_step(bump.st)==SQLITE_DONE && sqlite3_changes(db)==1;
    }
    if (!ok || !exec(db,"COMMIT")) {
      out.state=DraftEpochQuarantineLabState::persistence_fault;
      return out;
    }
    tx.committed=true;
    out.state=DraftEpochQuarantineLabState::quarantine_committed;
    out.saved_epoch=fence.epoch;
    out.quarantined_epoch=request.proposed_new_epoch;
    out.last_nonce=fence.nonce;
    out.last_sequence=verified.last_sequence;
    out.fence_generation=fence.generation+1;
    out.mission_run_id=verified.mission_run_id;
    out.signature=verified.signature;
    return out;
  }
 private:
  std::string file_;
};

} // namespace dyx::rpp
