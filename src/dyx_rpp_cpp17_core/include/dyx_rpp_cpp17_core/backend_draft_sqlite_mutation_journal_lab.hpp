#pragma once
// DYX Autonomous Phase 2H4Z: OFFLINE SQLite mutation journal prototype.
// No authentication, HTTP listener, PX4/DDS connection, actual mission backend,
// motor, spray or laser. This is NOT an exactly-once actuator transaction.
// It is an alternative future transaction boundary: do NOT reserve the same
// nonce using the H4W/H4X replay fence and this journal for one request.

#include <sqlite3.h>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace dyx::rpp {

struct DraftJournalLabProposal {
  std::string operator_session_id{};
  std::uint64_t session_epoch{0};
  std::uint64_t ingress_nonce{0};
  double received_sec{0.0};
  std::string mission_run_id{};
  std::uint64_t sequence{0};
  std::string route{};
  std::string canonical_request_json{};
};
struct DraftJournalLabKey {
  std::string operator_session_id{};
  std::uint64_t session_epoch{0};
  std::uint64_t ingress_nonce{0};
};
struct DraftJournalLabDecision {
  int http_status{500};
  std::string response_json{};
  // Snapshot is a complete opaque (not authenticated) mission draft record,
  // supplied by a future separately validated deterministic backend adapter.
  std::string complete_snapshot_json{};
  std::string stage{}; // PREVIEW or READY. NEVER STARTED.
};
struct DraftJournalLabEntry {
  std::string status{}; // PENDING, COMMITTED or DENIED
  std::string route{};
  std::string mission_run_id{};
  std::uint64_t sequence{0};
  std::string canonical_request_json{};
  DraftJournalLabDecision decision{};
};

namespace draft_journal_lab_detail {
inline bool serializable(std::uint64_t n) {
  return n <= static_cast<std::uint64_t>(std::numeric_limits<sqlite3_int64>::max());
}
inline bool token(const std::string& s) {
  if (s.empty() || s.size()>128) return false;
  for (const unsigned char c : s) {
    if (!((c>='A' && c<='Z') || (c>='a' && c<='z') ||
          (c>='0' && c<='9') || c=='_' || c=='-')) return false;
  }
  return true;
}
inline bool route(const std::string& s) {
  return s=="/api/v1/mission/upload" ||
         s=="/api/v1/mission/update" ||
         s=="/api/v1/mission/load";
}
inline bool bounded_json(const std::string& s, std::size_t max_bytes) {
  return !s.empty() && s.size()<=max_bytes &&
         s.find('\0')==std::string::npos &&
         s.front()=='{' && s.back()=='}';
}
inline bool proposal(const DraftJournalLabProposal& p) {
  return token(p.operator_session_id) && token(p.mission_run_id) &&
         serializable(p.session_epoch) && p.session_epoch>0 &&
         serializable(p.ingress_nonce) && p.ingress_nonce>0 &&
         serializable(p.sequence) && p.sequence>0 &&
         std::isfinite(p.received_sec) && p.received_sec>=0.0 &&
         route(p.route) && bounded_json(p.canonical_request_json,262144);
}
inline bool decision(const DraftJournalLabDecision& d, const std::string& route_name) {
  if (!bounded_json(d.response_json,4096)) return false;
  if (d.http_status==200)
    return bounded_json(d.complete_snapshot_json,262144) &&
           d.stage==(route_name=="/api/v1/mission/load"?"READY":"PREVIEW");
  return d.http_status>=400 && d.http_status<=599 &&
         d.complete_snapshot_json.empty() && d.stage.empty();
}
inline bool exec(sqlite3* db, const char* sql) {
  return sqlite3_exec(db,sql,nullptr,nullptr,nullptr)==SQLITE_OK;
}
struct Statement {
  sqlite3_stmt* st{nullptr};
  Statement(sqlite3* db,const char* sql) {
    if (sqlite3_prepare_v2(db,sql,-1,&st,nullptr)!=SQLITE_OK) st=nullptr;
  }
  ~Statement() { if (st) sqlite3_finalize(st); }
  Statement(const Statement&)=delete;
  Statement& operator=(const Statement&)=delete;
  explicit operator bool() const { return st!=nullptr; }
};
inline bool bind_text(sqlite3_stmt* st,int pos,const std::string& s) {
  return sqlite3_bind_text(st,pos,s.data(),static_cast<int>(s.size()),
                           SQLITE_TRANSIENT)==SQLITE_OK;
}
inline bool bind_integer(sqlite3_stmt* st,int pos,std::uint64_t v) {
  return serializable(v) && sqlite3_bind_int64(st,pos,static_cast<sqlite3_int64>(v))==SQLITE_OK;
}
inline bool text_col(sqlite3_stmt* st,int pos,std::string& output,std::size_t cap) {
  if (sqlite3_column_type(st,pos)!=SQLITE_TEXT) return false;
  const auto* p=sqlite3_column_text(st,pos);
  const auto count=sqlite3_column_bytes(st,pos);
  if (!p || count<0 || static_cast<std::size_t>(count)>cap) return false;
  output.assign(reinterpret_cast<const char*>(p),static_cast<std::size_t>(count));
  return output.find('\0')==std::string::npos;
}
inline bool pragma_check(sqlite3* db,const char* sql,const std::string& expected) {
  Statement stmt(db,sql);
  if (!stmt || sqlite3_step(stmt.st)!=SQLITE_ROW) return false;
  if (sqlite3_column_type(stmt.st,0)==SQLITE_INTEGER)
    return std::to_string(sqlite3_column_int(stmt.st,0))==expected;
  std::string actual;
  return text_col(stmt.st,0,actual,32) && actual==expected;
}
inline sqlite3* open_existing(const std::string& path) {
  if (path.empty() || path==":memory:" || path.rfind("file:",0)==0)
    throw std::invalid_argument("journal lab requires an existing SQLite file");
  sqlite3* db=nullptr;
  if (sqlite3_open_v2(path.c_str(),&db,
           SQLITE_OPEN_READWRITE|SQLITE_OPEN_FULLMUTEX,nullptr)!=SQLITE_OK) {
    if (db) sqlite3_close_v2(db);
    throw std::runtime_error("journal SQLite open denied");
  }
  sqlite3_busy_timeout(db,0); // never indefinitely block a safety decision
  return db;
}
inline bool policy(sqlite3* db) {
  return pragma_check(db,"PRAGMA journal_mode=WAL","wal") &&
         exec(db,"PRAGMA synchronous=FULL") &&
         pragma_check(db,"PRAGMA synchronous","2") &&
         pragma_check(db,"PRAGMA quick_check(1)","ok");
}
struct Fence {
  std::string session{};
  std::uint64_t epoch{0}, nonce{0}, generation{0};
  double sec{0.0};
};
inline bool read_fence(sqlite3* db,Fence& out) {
  Statement st(db,"SELECT session_id,epoch,nonce,received,generation "
                  "FROM replay_fence WHERE singleton=1");
  if (!st || sqlite3_step(st.st)!=SQLITE_ROW) return false;
  if (!text_col(st.st,0,out.session,128) || !token(out.session)) return false;
  for(int i : {1,2,4}) if (sqlite3_column_type(st.st,i)!=SQLITE_INTEGER) return false;
  if (sqlite3_column_type(st.st,3)!=SQLITE_FLOAT) return false;
  const auto epoch=sqlite3_column_int64(st.st,1);
  const auto nonce=sqlite3_column_int64(st.st,2);
  const auto generation=sqlite3_column_int64(st.st,4);
  if (epoch<=0 || nonce<0 || generation<=0) return false;
  out.epoch=static_cast<std::uint64_t>(epoch);
  out.nonce=static_cast<std::uint64_t>(nonce);
  out.generation=static_cast<std::uint64_t>(generation);
  out.sec=sqlite3_column_double(st.st,3);
  return std::isfinite(out.sec) && out.sec>=0.0 &&
         sqlite3_step(st.st)==SQLITE_DONE;
}
inline bool journal_schema_ok(sqlite3* db) {
  Statement meta(db,"SELECT version FROM dyx_journal_meta WHERE singleton=1");
  if (!meta || sqlite3_step(meta.st)!=SQLITE_ROW ||
      sqlite3_column_type(meta.st,0)!=SQLITE_INTEGER ||
      sqlite3_column_int(meta.st,0)!=1 || sqlite3_step(meta.st)!=SQLITE_DONE)
    return false;
  Statement st(db,"SELECT nonce,session_id,epoch,mission_run_id,sequence,route,"
                  "request_json,status,http_status,response_json,snapshot_json,stage "
                  "FROM dyx_mission_mutations LIMIT 0");
  Statement barrier(db,"SELECT session_id,old_epoch,new_epoch,nonce,old_generation,"
                       "new_generation,mission_run_id,signature,revision,"
                       "last_sequence,phase FROM dyx_mission_recovery_epoch_barrier LIMIT 0");
  Statement stage(db,"SELECT session_id,old_epoch,new_epoch,nonce,"
                       "generation,mission_run_id,signature,revision,"
                       "last_sequence,snapshot_json,phase "
                       "FROM dyx_mission_recovery_staging LIMIT 0");
  return static_cast<bool>(st) && static_cast<bool>(barrier) &&
         static_cast<bool>(stage);
}
} // namespace draft_journal_lab_detail

class NativeDraftSqliteMutationJournalLab final {
 public:
  // Manual, offline administrator action. The replay_fence must have already
  // been provisioned; this does NOT create it or accept remote enrollment.
  static bool provision_schema_for_existing_store(const std::string& path) {
    sqlite3* db=nullptr;
    try { db=draft_journal_lab_detail::open_existing(path); }
    catch (const std::exception&) { return false; }
    using namespace draft_journal_lab_detail;
    Fence fence;
    bool ok=policy(db) && read_fence(db,fence) && exec(db,"BEGIN IMMEDIATE");
    if (ok) {
      ok=exec(db,"CREATE TABLE IF NOT EXISTS dyx_journal_meta("
        "singleton INTEGER PRIMARY KEY CHECK(singleton=1),"
        "version INTEGER NOT NULL CHECK(version=1))") &&
         exec(db,"CREATE TABLE IF NOT EXISTS dyx_mission_mutations("
        "nonce INTEGER PRIMARY KEY CHECK(nonce>0),"
        "session_id TEXT NOT NULL,epoch INTEGER NOT NULL,"
        "mission_run_id TEXT NOT NULL,sequence INTEGER NOT NULL,"
        "route TEXT NOT NULL,request_json TEXT NOT NULL,"
        "status TEXT NOT NULL CHECK(status IN ('PENDING','COMMITTED','DENIED')) DEFAULT 'PENDING',"
        "http_status INTEGER NOT NULL DEFAULT 0,"
        "response_json TEXT NOT NULL DEFAULT '',"
        "snapshot_json TEXT NOT NULL DEFAULT '',"
        "stage TEXT NOT NULL DEFAULT '',"
        "UNIQUE(session_id,epoch,mission_run_id,sequence))") &&
         exec(db,"CREATE TABLE IF NOT EXISTS dyx_mission_recovery_epoch_barrier("
           "singleton INTEGER PRIMARY KEY CHECK(singleton=1),"
           "session_id TEXT NOT NULL,old_epoch INTEGER NOT NULL CHECK(old_epoch>0),"
           "new_epoch INTEGER NOT NULL CHECK(new_epoch>old_epoch),"
           "nonce INTEGER NOT NULL CHECK(nonce>0),"
           "old_generation INTEGER NOT NULL CHECK(old_generation>0),"
           "new_generation INTEGER NOT NULL CHECK(new_generation=old_generation+1),"
           "mission_run_id TEXT NOT NULL,signature TEXT NOT NULL,"
           "revision INTEGER NOT NULL CHECK(revision>0),"
           "last_sequence INTEGER NOT NULL CHECK(last_sequence>0),"
           "phase TEXT NOT NULL CHECK(phase='QUARANTINED'))") &&
         exec(db,"CREATE TABLE IF NOT EXISTS dyx_mission_recovery_staging("
           "singleton INTEGER PRIMARY KEY CHECK(singleton=1),"
           "session_id TEXT NOT NULL,old_epoch INTEGER NOT NULL CHECK(old_epoch>0),"
           "new_epoch INTEGER NOT NULL CHECK(new_epoch>old_epoch),"
           "nonce INTEGER NOT NULL CHECK(nonce>0),"
           "generation INTEGER NOT NULL CHECK(generation>0),"
           "mission_run_id TEXT NOT NULL,signature TEXT NOT NULL,"
           "revision INTEGER NOT NULL CHECK(revision>0),"
           "last_sequence INTEGER NOT NULL CHECK(last_sequence>0),"
           "snapshot_json TEXT NOT NULL,"
           "phase TEXT NOT NULL CHECK(phase='STAGED'))") &&
         exec(db,"INSERT OR IGNORE INTO dyx_journal_meta(singleton,version) VALUES(1,1)") &&
         journal_schema_ok(db);
      if (ok) ok=exec(db,"COMMIT");
      if (!ok) exec(db,"ROLLBACK");
    }
    sqlite3_close_v2(db);
    return ok;
  }

  explicit NativeDraftSqliteMutationJournalLab(const std::string& path) {
    db_=draft_journal_lab_detail::open_existing(path);
    if (!draft_journal_lab_detail::policy(db_) ||
        !draft_journal_lab_detail::journal_schema_ok(db_)) {
      sqlite3_close_v2(db_); db_=nullptr;
      throw std::runtime_error("missing, corrupt or incompatible journal schema");
    }
  }
  ~NativeDraftSqliteMutationJournalLab() { if(db_) sqlite3_close_v2(db_); }
  NativeDraftSqliteMutationJournalLab(const NativeDraftSqliteMutationJournalLab&)=delete;
  NativeDraftSqliteMutationJournalLab& operator=(const NativeDraftSqliteMutationJournalLab&)=delete;

  // The two updates (replay fence + journal PENDING request) commit atomically.
  // Must NOT execute backend mutation automatically. PENDING blocks another POST.
  bool begin(const DraftJournalLabProposal& p) {
    using namespace draft_journal_lab_detail;
    if (!proposal(p)) return false;
    std::lock_guard<std::mutex> lock(mu_);
    if (!exec(db_,"BEGIN IMMEDIATE")) return false;
    // H5H permanent quarantine barrier: deny ALL journal writes, including
    // proposals carrying the freshly rotated epoch. Missing table fails shut.
    Statement barrier(db_,"SELECT 1 FROM dyx_mission_recovery_epoch_barrier LIMIT 1");
    if (!barrier || sqlite3_step(barrier.st)!=SQLITE_DONE) {
      exec(db_,"ROLLBACK"); return false;
    }
    Fence f;
    bool ok=read_fence(db_,f) && f.session==p.operator_session_id &&
       f.epoch==p.session_epoch && p.ingress_nonce>f.nonce &&
       p.received_sec>=f.sec && serializable(f.generation+1) &&
       f.generation < static_cast<std::uint64_t>(std::numeric_limits<sqlite3_int64>::max());
    if (ok) {
      Statement pending(db_,"SELECT 1 FROM dyx_mission_mutations "
                            "WHERE status='PENDING' LIMIT 1");
      ok=pending && sqlite3_step(pending.st)==SQLITE_DONE;
    }
    if (ok) {
      Statement update(db_,"UPDATE replay_fence SET nonce=?,received=?,generation=?"
                           " WHERE singleton=1");
      ok=update && bind_integer(update.st,1,p.ingress_nonce) &&
         sqlite3_bind_double(update.st,2,p.received_sec)==SQLITE_OK &&
         bind_integer(update.st,3,f.generation+1) &&
         sqlite3_step(update.st)==SQLITE_DONE && sqlite3_changes(db_)==1;
    }
    if (ok) {
      Statement insert(db_,"INSERT INTO dyx_mission_mutations("
        "nonce,session_id,epoch,mission_run_id,sequence,route,request_json)"
        " VALUES(?,?,?,?,?,?,?)");
      ok=insert && bind_integer(insert.st,1,p.ingress_nonce) &&
         bind_text(insert.st,2,p.operator_session_id) &&
         bind_integer(insert.st,3,p.session_epoch) &&
         bind_text(insert.st,4,p.mission_run_id) &&
         bind_integer(insert.st,5,p.sequence) &&
         bind_text(insert.st,6,p.route) &&
         bind_text(insert.st,7,p.canonical_request_json) &&
         sqlite3_step(insert.st)==SQLITE_DONE && sqlite3_changes(db_)==1;
    }
    if (ok) ok=exec(db_,"COMMIT");
    if (!ok) exec(db_,"ROLLBACK");
    return ok;
  }

  // This is a separate transaction after a hypothetical backend decision.
  // An intervening crash leaves PENDING: fail closed, require operator review.
  // The existing in-memory mission backend is NOT recovered by this lab.
  bool conclude(const DraftJournalLabKey& key,const DraftJournalLabDecision& decision) {
    using namespace draft_journal_lab_detail;
    if (!token(key.operator_session_id) || !serializable(key.session_epoch) ||
        key.session_epoch==0 || !serializable(key.ingress_nonce) ||
        key.ingress_nonce==0) return false;
    std::lock_guard<std::mutex> lock(mu_);
    if (!exec(db_,"BEGIN IMMEDIATE")) return false;
    // No late conclusion may mutate the journal after epoch quarantine.
    Statement barrier(db_,"SELECT 1 FROM dyx_mission_recovery_epoch_barrier LIMIT 1");
    if (!barrier || sqlite3_step(barrier.st)!=SQLITE_DONE) {
      exec(db_,"ROLLBACK"); return false;
    }
    Statement check(db_,"SELECT route FROM dyx_mission_mutations "
                        "WHERE nonce=? AND session_id=? AND epoch=? AND status='PENDING'");
    std::string route_name;
    bool ok=check && bind_integer(check.st,1,key.ingress_nonce) &&
        bind_text(check.st,2,key.operator_session_id) &&
        bind_integer(check.st,3,key.session_epoch) &&
        sqlite3_step(check.st)==SQLITE_ROW &&
        text_col(check.st,0,route_name,64) &&
        decision_ok(decision,route_name);
    if (ok) {
      Statement update(db_,"UPDATE dyx_mission_mutations SET status=?,"
         "http_status=?,response_json=?,snapshot_json=?,stage=?"
         " WHERE nonce=? AND session_id=? AND epoch=? AND status='PENDING'");
      const auto new_status=decision.http_status==200?"COMMITTED":"DENIED";
      ok=update && bind_text(update.st,1,new_status) &&
         sqlite3_bind_int(update.st,2,decision.http_status)==SQLITE_OK &&
         bind_text(update.st,3,decision.response_json) &&
         bind_text(update.st,4,decision.complete_snapshot_json) &&
         bind_text(update.st,5,decision.stage) &&
         bind_integer(update.st,6,key.ingress_nonce) &&
         bind_text(update.st,7,key.operator_session_id) &&
         bind_integer(update.st,8,key.session_epoch) &&
         sqlite3_step(update.st)==SQLITE_DONE && sqlite3_changes(db_)==1;
    }
    if (ok) ok=exec(db_,"COMMIT");
    if (!ok) exec(db_,"ROLLBACK");
    return ok;
  }

  // Inspect is a lab-only read. Real access MUST independently authorize the
  // caller and verify session/lease; no such authenticator is supplied here.
  bool inspect(const DraftJournalLabKey& key,DraftJournalLabEntry& output) {
    using namespace draft_journal_lab_detail;
    if (!token(key.operator_session_id) || !serializable(key.session_epoch) ||
        key.session_epoch==0 || !serializable(key.ingress_nonce) ||
        key.ingress_nonce==0) return false;
    std::lock_guard<std::mutex> lock(mu_);
    Statement st(db_,"SELECT status,route,mission_run_id,sequence,request_json,"
                     "http_status,response_json,snapshot_json,stage "
                     "FROM dyx_mission_mutations WHERE nonce=? AND session_id=? AND epoch=?");
    if (!st || !bind_integer(st.st,1,key.ingress_nonce) ||
        !bind_text(st.st,2,key.operator_session_id) ||
        !bind_integer(st.st,3,key.session_epoch) ||
        sqlite3_step(st.st)!=SQLITE_ROW) return false;
    DraftJournalLabEntry e;
    if (!text_col(st.st,0,e.status,16) ||
        !text_col(st.st,1,e.route,64) ||
        !text_col(st.st,2,e.mission_run_id,128) ||
        sqlite3_column_type(st.st,3)!=SQLITE_INTEGER ||
        sqlite3_column_int64(st.st,3)<=0 ||
        !text_col(st.st,4,e.canonical_request_json,262144) ||
        sqlite3_column_type(st.st,5)!=SQLITE_INTEGER ||
        !text_col(st.st,6,e.decision.response_json,4096) ||
        !text_col(st.st,7,e.decision.complete_snapshot_json,262144) ||
        !text_col(st.st,8,e.decision.stage,16)) return false;
    e.sequence=static_cast<std::uint64_t>(sqlite3_column_int64(st.st,3));
    e.decision.http_status=sqlite3_column_int(st.st,5);
    if (!(e.status=="PENDING" || e.status=="COMMITTED" || e.status=="DENIED")) return false;
    if (e.status=="PENDING" && (e.decision.http_status!=0 ||
        !e.decision.response_json.empty() ||
        !e.decision.complete_snapshot_json.empty() || !e.decision.stage.empty()))
      return false;
    if (e.status!="PENDING" && (!decision_ok(e.decision,e.route) ||
        (e.status=="COMMITTED")!=(e.decision.http_status==200))) return false;
    output=std::move(e);
    return sqlite3_step(st.st)==SQLITE_DONE;
  }

  // Conservative startup gate: even a completed journal record indicates
  // process-local mission backend state has not been restored. Never silently
  // initialize an empty NativeBackendMissionUpload against recorded history.
  bool has_history(bool& exists) {
    std::lock_guard<std::mutex> lock(mu_);
    draft_journal_lab_detail::Statement stmt(db_,
        "SELECT 1 FROM dyx_mission_mutations LIMIT 1");
    if (!stmt) return false;
    const int step=sqlite3_step(stmt.st);
    if (step!=SQLITE_ROW && step!=SQLITE_DONE) return false;
    exists=step==SQLITE_ROW;
    return true;
  }

  bool has_pending(bool& pending) {
    using namespace draft_journal_lab_detail;
    std::lock_guard<std::mutex> lock(mu_);
    Statement stmt(db_,"SELECT 1 FROM dyx_mission_mutations WHERE status='PENDING' LIMIT 1");
    if (!stmt) return false;
    const int result=sqlite3_step(stmt.st);
    if (result!=SQLITE_ROW && result!=SQLITE_DONE) return false;
    pending=result==SQLITE_ROW;
    return true;
  }
 private:
  static bool decision_ok(const DraftJournalLabDecision& d,const std::string& route) {
    return draft_journal_lab_detail::decision(d,route);
  }
  sqlite3* db_{nullptr};
  std::mutex mu_{};
};

} // namespace dyx::rpp
