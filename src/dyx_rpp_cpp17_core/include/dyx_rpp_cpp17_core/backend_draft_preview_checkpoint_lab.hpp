#pragma once
// DYX Autonomous Phase 2H5E — OFFLINE, non-actuating, versioned SQLite
// checkpoint of a previously COMMITTED PREVIEW. This intentionally does NOT
// recover NativeBackendMissionUpload, unquarantine journal writers, install a
// mission into the gateway, or grant motion/LOAD/START/RESUME authority.
// Authenticated/authorized identity here is asserted by a trusted caller;
// this class does not independently authenticate or obtain PX4 evidence.
// Checkpoint copying is a separate transaction from journal conclusion:
// a snapshot without a checkpoint remains valid journal history for review.

#include "dyx_rpp_cpp17_core/backend_draft_history_screen_lab.hpp"
#include <sqlite3.h>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace dyx::rpp {

enum class DraftPreviewCheckpointLabState {
  access_denied, unavailable, rejected_history, not_checkpointed,
  checkpoint_mismatch, sealed_preview, verified_preview
};
struct DraftPreviewCheckpointLabResult {
  DraftPreviewCheckpointLabState state{DraftPreviewCheckpointLabState::unavailable};
  std::uint64_t schema_version{0}, revision{0}, ingress_nonce{0};
  std::string mission_run_id{},signature{};
  // A copy for a read-only operator/map review. No backend is modified.
  std::optional<PreparedPathSnapshot> review_snapshot{};
  bool can_restore{false},can_load{false},can_start_or_resume{false};
  bool hardware_command_emitted{false};
};

namespace preview_checkpoint_detail {
using namespace draft_journal_lab_detail;
constexpr const char* kSchema =
    "CREATE TABLE IF NOT EXISTS dyx_preview_checkpoint ("
    "singleton INTEGER PRIMARY KEY CHECK(singleton=1),"
    "schema_version INTEGER NOT NULL CHECK(schema_version=1),"
    "session_id TEXT NOT NULL, epoch INTEGER NOT NULL,"
    "nonce INTEGER NOT NULL, mission_run_id TEXT NOT NULL,"
    "revision INTEGER NOT NULL, signature TEXT NOT NULL,"
    "snapshot_json TEXT NOT NULL)";
struct DbHandle {
  sqlite3* db{nullptr};
  explicit DbHandle(const std::string& file,bool writable) {
    if (file.empty() || file==":memory:" || file.rfind("file:",0)==0) return;
    const auto flags=(writable?SQLITE_OPEN_READWRITE:SQLITE_OPEN_READONLY)|SQLITE_OPEN_FULLMUTEX;
    if (sqlite3_open_v2(file.c_str(),&db,flags,nullptr)!=SQLITE_OK) {
      if (db) sqlite3_close_v2(db);
      db=nullptr;
    }
    if (db) sqlite3_busy_timeout(db,0); // never delay a STOP/safety path
  }
  ~DbHandle(){if(db) sqlite3_close_v2(db);}
  explicit operator bool()const{return db!=nullptr;}
};
struct Tx {
  sqlite3* db;
  bool done{false};
  explicit Tx(sqlite3* handle,bool writable) : db(handle) {
    done=!exec(db,writable?"BEGIN IMMEDIATE":"BEGIN");
  }
  ~Tx(){if(!done)exec(db,"ROLLBACK");}
  bool active()const{return !done;}
  bool commit(){if(done||!exec(db,"COMMIT"))return false;done=true;return true;}
};
struct Record {
  std::uint64_t nonce{0};
  std::string status{},route{},snapshot_json{},run{};
};
inline bool latest(sqlite3* db,Record& out) {
  Statement st(db,"SELECT nonce,status,route,snapshot_json,mission_run_id "
                  "FROM dyx_mission_mutations ORDER BY nonce DESC LIMIT 1");
  if(!st || sqlite3_step(st.st)!=SQLITE_ROW ||
     sqlite3_column_type(st.st,0)!=SQLITE_INTEGER) return false;
  const auto nonce=sqlite3_column_int64(st.st,0);
  if(nonce<=0 || !text_col(st.st,1,out.status,16) ||
     !text_col(st.st,2,out.route,64) ||
     !text_col(st.st,3,out.snapshot_json,262144) ||
     !text_col(st.st,4,out.run,128))return false;
  out.nonce=static_cast<std::uint64_t>(nonce);
  return true;
}
inline bool parse_final(const Record& last,const DraftHistoryScreenLabResult& screen,
                        const DraftReconciliationLabIdentity& identity,
                        draft_audit_lab_detail::StrictSnapshotDecoder::Parsed& parsed) {
  if(last.status!="COMMITTED" ||
     (last.route!=kBackendMissionUploadPath && last.route!=kBackendMissionUpdatePath) ||
     last.run!=screen.mission_run_id ||
     !draft_audit_lab_detail::StrictSnapshotDecoder(last.snapshot_json).decode(parsed) ||
     parsed.loaded || parsed.operator_session!=identity.operator_session_id ||
     parsed.run!=last.run || parsed.revision!=screen.revision ||
     parsed.snapshot.signature!=screen.signature ||
     parsed.snapshot.marking_points.size()!=screen.marking_count ||
     parsed.snapshot.navigation_points.size()!=screen.navigation_count)
    return false;
  BackendMissionUploadRequest req;
  req.path=last.route;req.mission_run_id=last.run;
  req.principal.operator_session_id=identity.operator_session_id;
  BackendMissionUploadResponse response;
  response.http_status=200;response.revision=parsed.revision;
  response.signature=parsed.snapshot.signature;
  response.marking_count=parsed.snapshot.marking_points.size();
  response.navigation_count=parsed.snapshot.navigation_points.size();
  response.draft_ready=true;
  std::string roundtrip;
  return journaled_upload_lab_detail::json_snapshot(response,req,
             &parsed.snapshot,parsed.mode,roundtrip) && roundtrip==last.snapshot_json;
}
inline bool matching_fence(sqlite3* db,const DraftReconciliationLabIdentity& identity,
                           std::uint64_t nonce) {
  Fence fence;
  return read_fence(db,fence) &&
         fence.session==identity.operator_session_id &&
         fence.epoch==identity.session_epoch && fence.nonce==nonce;
}
} // namespace preview_checkpoint_detail

class NativeOfflineDraftPreviewCheckpointLab final {
 public:
  explicit NativeOfflineDraftPreviewCheckpointLab(std::string existing_file)
    : file_(std::move(existing_file)) {}

  DraftPreviewCheckpointLabResult seal_preview(const DraftReconciliationLabIdentity& id)const {
    using namespace preview_checkpoint_detail;
    DraftPreviewCheckpointLabResult out;
    if (!authorized(id)) {out.state=DraftPreviewCheckpointLabState::access_denied;return out;}
    DbHandle h(file_,true);
    if(!h || !pragma_check(h.db,"PRAGMA journal_mode","wal") ||
       !pragma_check(h.db,"PRAGMA quick_check(1)","ok") ||
       !exec(h.db,"PRAGMA synchronous=FULL"))return out;
    Tx tx(h.db,true);
    if(!tx.active())return out;
    // BEGIN IMMEDIATE holds SQLite's single-writer lock across screening,
    // latest-journal comparison and checkpoint insertion.
    const auto screen=NativeOfflineDraftHistoryScreenLab(file_).inspect(id);
    if(screen.state!=DraftHistoryScreenLabState::consistent_preview_history ||
       !screen.history_consistency_verified) {
      out.state=DraftPreviewCheckpointLabState::rejected_history;return out;
    }
    Record last;
    draft_audit_lab_detail::StrictSnapshotDecoder::Parsed parsed;
    if(!latest(h.db,last) || !matching_fence(h.db,id,last.nonce) ||
       !parse_final(last,screen,id,parsed)) {
      out.state=DraftPreviewCheckpointLabState::rejected_history;return out;
    }
    if(!exec(h.db,kSchema))return out;
    Statement st(h.db,"INSERT OR IGNORE INTO dyx_preview_checkpoint "
       "(singleton,schema_version,session_id,epoch,nonce,mission_run_id,revision,signature,snapshot_json) "
       "VALUES(1,1,?,?,?,?,?,?,?)");
    if(!st || !bind_text(st.st,1,id.operator_session_id) ||
       !bind_integer(st.st,2,id.session_epoch) ||
       !bind_integer(st.st,3,last.nonce) ||
       !bind_text(st.st,4,last.run) ||
       !bind_integer(st.st,5,parsed.revision) ||
       !bind_text(st.st,6,parsed.snapshot.signature) ||
       !bind_text(st.st,7,last.snapshot_json) || sqlite3_step(st.st)!=SQLITE_DONE)return out;
    // A historical checkpoint may never be overwritten by a newer revision;
    // explicit operator-led archival/retirement is needed instead.
    if (!identical(h.db,id,last,parsed)) {
      out.state=DraftPreviewCheckpointLabState::checkpoint_mismatch;return out;
    }
    if(!tx.commit())return out;
    out.state=DraftPreviewCheckpointLabState::sealed_preview;
    out.schema_version=1;out.revision=parsed.revision;out.ingress_nonce=last.nonce;
    out.mission_run_id=last.run;out.signature=parsed.snapshot.signature;
    return out;
  }

  DraftPreviewCheckpointLabResult inspect_preview(const DraftReconciliationLabIdentity& id)const {
    using namespace preview_checkpoint_detail;
    DraftPreviewCheckpointLabResult out;
    if (!authorized(id)) {out.state=DraftPreviewCheckpointLabState::access_denied;return out;}
    DbHandle h(file_,false);
    if(!h || !exec(h.db,"PRAGMA query_only=ON") ||
       !pragma_check(h.db,"PRAGMA journal_mode","wal") ||
       !pragma_check(h.db,"PRAGMA quick_check(1)","ok"))return out;
    Tx tx(h.db,false);
    if(!tx.active())return out;
    // Pin the SQLite read snapshot before asking another connection to screen.
    Fence f;
    if(!read_fence(h.db,f))return out;
    const auto screen=NativeOfflineDraftHistoryScreenLab(file_).inspect(id);
    if(screen.state!=DraftHistoryScreenLabState::consistent_preview_history ||
       !screen.history_consistency_verified) {
      out.state=DraftPreviewCheckpointLabState::rejected_history;return out;
    }
    Record last;
    draft_audit_lab_detail::StrictSnapshotDecoder::Parsed parsed;
    if(!latest(h.db,last) || !matching_fence(h.db,id,last.nonce) ||
       !parse_final(last,screen,id,parsed)){
      out.state=DraftPreviewCheckpointLabState::rejected_history;return out;
    }
    Statement st(h.db,"SELECT schema_version,session_id,epoch,nonce,mission_run_id,"
      "revision,signature,snapshot_json FROM dyx_preview_checkpoint WHERE singleton=1");
    if(!st)return out;
    const auto step=sqlite3_step(st.st);
    if(step==SQLITE_DONE) {out.state=DraftPreviewCheckpointLabState::not_checkpointed;return out;}
    if(step!=SQLITE_ROW || !identical_row(st.st,id,last,parsed)) {
      out.state=DraftPreviewCheckpointLabState::checkpoint_mismatch;return out;
    }
    out.state=DraftPreviewCheckpointLabState::verified_preview;
    out.schema_version=1;out.revision=parsed.revision;out.ingress_nonce=last.nonce;
    out.mission_run_id=last.run;out.signature=parsed.snapshot.signature;
    out.review_snapshot=std::move(parsed.snapshot); // review copy ONLY
    return out;
  }
 private:
  using Parsed=draft_audit_lab_detail::StrictSnapshotDecoder::Parsed;
  static bool authorized(const DraftReconciliationLabIdentity& id) {
    return id.authenticated && id.can_read_mission && id.fresh_session_evidence &&
           draft_journal_lab_detail::token(id.operator_session_id) &&
           id.session_epoch>0 && draft_journal_lab_detail::serializable(id.session_epoch);
  }
  static bool identical_row(sqlite3_stmt* st,const DraftReconciliationLabIdentity& id,
                            const preview_checkpoint_detail::Record& last,const Parsed& p) {
    using namespace draft_journal_lab_detail;
    std::string session,run,sig,json;
    return sqlite3_column_type(st,0)==SQLITE_INTEGER && sqlite3_column_int64(st,0)==1 &&
           sqlite3_column_type(st,2)==SQLITE_INTEGER &&
           sqlite3_column_int64(st,2)==static_cast<sqlite3_int64>(id.session_epoch) &&
           sqlite3_column_type(st,3)==SQLITE_INTEGER &&
           sqlite3_column_int64(st,3)==static_cast<sqlite3_int64>(last.nonce) &&
           sqlite3_column_type(st,5)==SQLITE_INTEGER &&
           sqlite3_column_int64(st,5)==static_cast<sqlite3_int64>(p.revision) &&
           text_col(st,1,session,128) && session==id.operator_session_id &&
           text_col(st,4,run,128) && run==last.run &&
           text_col(st,6,sig,64) && sig==p.snapshot.signature &&
           text_col(st,7,json,262144) && json==last.snapshot_json;
  }
  static bool identical(sqlite3* db,const DraftReconciliationLabIdentity& id,
                        const preview_checkpoint_detail::Record& last,const Parsed& p) {
    draft_journal_lab_detail::Statement st(db,"SELECT schema_version,session_id,epoch,nonce,"
        "mission_run_id,revision,signature,snapshot_json FROM dyx_preview_checkpoint WHERE singleton=1");
    return st && sqlite3_step(st.st)==SQLITE_ROW && identical_row(st.st,id,last,p);
  }
  std::string file_;
};
} // namespace dyx::rpp
