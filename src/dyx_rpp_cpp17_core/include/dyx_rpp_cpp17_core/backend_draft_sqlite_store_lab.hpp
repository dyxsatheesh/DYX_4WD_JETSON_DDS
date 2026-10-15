#pragma once
// DYX Autonomous Phase 2H4X: OPTIONAL Jetson/Linux SQLite replay-store PILOT.
// Explicit provisioning only; fail-closed on errors. SQLite transactions are
// durable under its documented filesystem/power-failure assumptions, but this
// is NOT a complete production security or mission transaction system.
// No HTTP server, crypto, DDS/PX4 reader, mission actuator or motor commands.

#include "dyx_rpp_cpp17_core/backend_draft_restart_fence_lab.hpp"
#include <sqlite3.h>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace dyx::rpp {

namespace draft_sqlite_detail {
inline bool signed_ok(std::uint64_t n) {
  return n <= static_cast<std::uint64_t>(std::numeric_limits<sqlite3_int64>::max());
}
inline bool storeable(const DraftRestartLabRecord& r) {
  return draft_restart_lab_record_valid(r) && signed_ok(r.session_epoch) &&
         signed_ok(r.last_ingress_nonce) && signed_ok(r.generation);
}
inline bool run(sqlite3* db, const char* sql) {
  return sqlite3_exec(db, sql, nullptr, nullptr, nullptr) == SQLITE_OK;
}
struct Stmt final {
  sqlite3_stmt* handle{nullptr};
  Stmt(sqlite3* db, const char* sql) {
    if (sqlite3_prepare_v2(db, sql, -1, &handle, nullptr) != SQLITE_OK)
      handle=nullptr;
  }
  ~Stmt() { if (handle) sqlite3_finalize(handle); }
  Stmt(const Stmt&)=delete;
  Stmt& operator=(const Stmt&)=delete;
  explicit operator bool() const { return handle != nullptr; }
};
inline bool bind_record(sqlite3_stmt* st, const DraftRestartLabRecord& r) {
  if (!storeable(r)) return false;
  const int len=static_cast<int>(r.operator_session_id.size());
  if (sqlite3_bind_text(st,1,r.operator_session_id.data(),len,SQLITE_TRANSIENT)!=SQLITE_OK)
    return false;
  return sqlite3_bind_int64(st,2,static_cast<sqlite3_int64>(r.session_epoch))==SQLITE_OK &&
         sqlite3_bind_int64(st,3,static_cast<sqlite3_int64>(r.last_ingress_nonce))==SQLITE_OK &&
         sqlite3_bind_double(st,4,r.last_received_sec)==SQLITE_OK &&
         sqlite3_bind_int64(st,5,static_cast<sqlite3_int64>(r.generation))==SQLITE_OK;
}
inline bool read(sqlite3* db, DraftRestartLabRecord& output) {
  Stmt st(db,"SELECT session_id, epoch, nonce, received, generation "
             "FROM replay_fence WHERE singleton=1");
  if (!st || sqlite3_step(st.handle)!=SQLITE_ROW) return false;
  for (int i=0;i<5;++i)
    if (sqlite3_column_type(st.handle,i)==SQLITE_NULL) return false;
  if (sqlite3_column_type(st.handle,0)!=SQLITE_TEXT ||
      sqlite3_column_type(st.handle,1)!=SQLITE_INTEGER ||
      sqlite3_column_type(st.handle,2)!=SQLITE_INTEGER ||
      sqlite3_column_type(st.handle,3)!=SQLITE_FLOAT ||
      sqlite3_column_type(st.handle,4)!=SQLITE_INTEGER) return false;
  const auto* p=sqlite3_column_text(st.handle,0);
  const int len=sqlite3_column_bytes(st.handle,0);
  if (!p || len<=0 || len>128) return false;
  const auto epoch=sqlite3_column_int64(st.handle,1);
  const auto nonce=sqlite3_column_int64(st.handle,2);
  const auto gen=sqlite3_column_int64(st.handle,4);
  if (epoch<=0 || nonce<0 || gen<=0) return false;
  DraftRestartLabRecord record;
  record.operator_session_id.assign(reinterpret_cast<const char*>(p),static_cast<std::size_t>(len));
  record.session_epoch=static_cast<std::uint64_t>(epoch);
  record.last_ingress_nonce=static_cast<std::uint64_t>(nonce);
  record.last_received_sec=sqlite3_column_double(st.handle,3);
  record.generation=static_cast<std::uint64_t>(gen);
  if (!storeable(record) || sqlite3_step(st.handle)!=SQLITE_DONE) return false;
  output=std::move(record);
  return true;
}
inline bool pragma_value(sqlite3* db,const char* sql,const char* expected) {
  Stmt st(db,sql);
  if (!st || sqlite3_step(st.handle)!=SQLITE_ROW) return false;
  const unsigned char* value=sqlite3_column_text(st.handle,0);
  return value && std::string(reinterpret_cast<const char*>(value))==expected;
}
inline sqlite3* open(const std::string& path,int flags) {
  if (path.empty() || path==":memory:" || path.rfind("file:",0)==0)
    throw std::invalid_argument("SQLite replay lab requires a filesystem path");
  sqlite3* db=nullptr;
  if (sqlite3_open_v2(path.c_str(),&db,flags|SQLITE_OPEN_FULLMUTEX,nullptr)!=SQLITE_OK) {
    if (db) sqlite3_close_v2(db);
    throw std::runtime_error("cannot open replay SQLite database");
  }
  // Never allow requests to wait indefinitely for a cross-process writer.
  sqlite3_busy_timeout(db,0);
  return db;
}
inline bool configure(sqlite3* db) {
  return pragma_value(db,"PRAGMA journal_mode=WAL","wal") &&
         run(db,"PRAGMA synchronous=FULL") &&
         pragma_value(db,"PRAGMA synchronous","2") &&
         pragma_value(db,"PRAGMA quick_check(1)","ok");
}
} // namespace draft_sqlite_detail

class NativeDraftSqliteStoreLab final : public DraftRestartLabStore {
 public:
  // Only an explicit, operator-controlled provisioning action may call this.
  // The target must NOT already exist; do not use in an HTTP request handler.
  // Path, permissions, backup and credential policy need independent audit.
  static bool provision_new(const std::string& path,
                            const DraftRestartLabRecord& initial) {
    if (!draft_sqlite_detail::storeable(initial) ||
        initial.last_ingress_nonce!=0 || initial.generation!=1) return false;
    // SQLite CREATE is NOT exclusive; caller must control provisioning lock
    // and directory ACLs. Existence is checked first to prevent overwrite.
    {
      sqlite3* existing=nullptr;
      const int rc=sqlite3_open_v2(path.c_str(),&existing,
           SQLITE_OPEN_READWRITE|SQLITE_OPEN_FULLMUTEX,nullptr);
      if (existing) sqlite3_close_v2(existing);
      if (rc==SQLITE_OK) return false;
    }
    sqlite3* db=nullptr;
    try { db=draft_sqlite_detail::open(path,SQLITE_OPEN_READWRITE|SQLITE_OPEN_CREATE); }
    catch (const std::exception&) { return false; }
    const bool configured=draft_sqlite_detail::configure(db);
    const bool schema=configured && draft_sqlite_detail::run(db,
      "CREATE TABLE IF NOT EXISTS replay_fence("
      " singleton INTEGER PRIMARY KEY CHECK(singleton=1),"
      " session_id TEXT NOT NULL, epoch INTEGER NOT NULL CHECK(epoch>0),"
      " nonce INTEGER NOT NULL CHECK(nonce>=0),"
      " received REAL NOT NULL CHECK(received>=0),"
      " generation INTEGER NOT NULL CHECK(generation>0))");
    bool inserted=false;
    if (schema && draft_sqlite_detail::run(db,"BEGIN IMMEDIATE")) {
      draft_sqlite_detail::Stmt st(db,
        "INSERT INTO replay_fence(singleton,session_id,epoch,nonce,received,generation)"
        " VALUES(1,?,?,?,?,?)");
      inserted=st && draft_sqlite_detail::bind_record(st.handle,initial) &&
               sqlite3_step(st.handle)==SQLITE_DONE;
      inserted=inserted && sqlite3_changes(db)==1;
      if (inserted) inserted=draft_sqlite_detail::run(db,"COMMIT");
      if (!inserted) draft_sqlite_detail::run(db,"ROLLBACK");
    }
    sqlite3_close_v2(db);
    return inserted;
  }

  explicit NativeDraftSqliteStoreLab(const std::string& path) {
    db_=draft_sqlite_detail::open(path,SQLITE_OPEN_READWRITE);
    if (!draft_sqlite_detail::configure(db_)) {
      sqlite3_close_v2(db_); db_=nullptr;
      throw std::runtime_error("replay SQLite integrity or WAL/FULL policy failure");
    }
    // A missing or corrupt record must DENY on load, never auto-provision.
  }
  ~NativeDraftSqliteStoreLab() override { if (db_) sqlite3_close_v2(db_); }
  NativeDraftSqliteStoreLab(const NativeDraftSqliteStoreLab&)=delete;
  NativeDraftSqliteStoreLab& operator=(const NativeDraftSqliteStoreLab&)=delete;

  bool load_existing(DraftRestartLabRecord& out) override {
    std::lock_guard<std::mutex> lock(mu_);
    return draft_sqlite_detail::read(db_,out);
  }
  bool compare_exchange_durable(const DraftRestartLabRecord& expected,
                                const DraftRestartLabRecord& next) override {
    std::lock_guard<std::mutex> lock(mu_);
    if (!draft_sqlite_detail::storeable(expected) ||
        !draft_sqlite_detail::storeable(next) ||
        next.operator_session_id!=expected.operator_session_id ||
        next.session_epoch!=expected.session_epoch ||
        expected.generation==std::numeric_limits<std::uint64_t>::max() ||
        next.generation!=expected.generation+1 ||
        next.last_ingress_nonce<=expected.last_ingress_nonce ||
        next.last_received_sec<expected.last_received_sec) return false;
    if (!draft_sqlite_detail::run(db_,"BEGIN IMMEDIATE")) return false;
    DraftRestartLabRecord current;
    bool ok=draft_sqlite_detail::read(db_,current) &&
            draft_restart_lab_record_equal(current,expected);
    if (ok) {
      draft_sqlite_detail::Stmt st(db_,
        "UPDATE replay_fence SET session_id=?,epoch=?,nonce=?,received=?,generation=?"
        " WHERE singleton=1");
      ok=st && draft_sqlite_detail::bind_record(st.handle,next) &&
         sqlite3_step(st.handle)==SQLITE_DONE && sqlite3_changes(db_)==1;
    }
    if (ok) ok=draft_sqlite_detail::run(db_,"COMMIT");
    if (!ok) draft_sqlite_detail::run(db_,"ROLLBACK");
    return ok;
  }
 private:
  sqlite3* db_{nullptr};
  std::mutex mu_{};
};

} // namespace dyx::rpp
