// DYX Autonomous Phase 2H4Z. Offline SQLite journal tests, no robot IO.
#include "dyx_rpp_cpp17_core/backend_draft_sqlite_mutation_journal_lab.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
using namespace dyx::rpp;
namespace fs=std::filesystem;

struct TempJournalDb {
  fs::path directory,path;
  explicit TempJournalDb(const std::string& name) {
    auto now=std::chrono::steady_clock::now().time_since_epoch().count();
    directory=fs::temp_directory_path()/("dyx_h4z_"+name+"_"+std::to_string(now));
    assert(fs::create_directory(directory));
    path=directory/"rover.sqlite";
  }
  ~TempJournalDb() { std::error_code ignored; fs::remove_all(directory,ignored); }
};

// Reproduce precisely the singleton H4X replay_fence schema + initial row,
// without loading PX4/Android or a robot backend. No remote provisioning.
static void offline_preprovision(const fs::path& path) {
  sqlite3* db=nullptr;
  assert(sqlite3_open_v2(path.string().c_str(),&db,
      SQLITE_OPEN_READWRITE|SQLITE_OPEN_CREATE|SQLITE_OPEN_FULLMUTEX,nullptr)==SQLITE_OK);
  auto run=[&](const char* sql) { assert(sqlite3_exec(db,sql,nullptr,nullptr,nullptr)==SQLITE_OK); };
  run("PRAGMA journal_mode=WAL");
  run("PRAGMA synchronous=FULL");
  run("CREATE TABLE replay_fence(singleton INTEGER PRIMARY KEY CHECK(singleton=1),"
      "session_id TEXT NOT NULL,epoch INTEGER NOT NULL CHECK(epoch>0),"
      "nonce INTEGER NOT NULL CHECK(nonce>=0),received REAL NOT NULL CHECK(received>=0),"
      "generation INTEGER NOT NULL CHECK(generation>0))");
  run("INSERT INTO replay_fence VALUES(1,'SESSION_H4R_OPERATOR',3,0,0.0,1)");
  assert(sqlite3_close_v2(db)==SQLITE_OK);
}
static void provision(TempJournalDb& db) {
  offline_preprovision(db.path);
  assert(NativeDraftSqliteMutationJournalLab::provision_schema_for_existing_store(db.path.string()));
}
static DraftJournalLabProposal proposal(std::uint64_t nonce,std::uint64_t sequence,
                                        std::string route="/api/v1/mission/upload") {
  return {"SESSION_H4R_OPERATOR",3,nonce,10.0+static_cast<double>(nonce),
          "RUN_H4Z_1",sequence,std::move(route),"{\"sequence\":"+std::to_string(sequence)+"}"};
}
static DraftJournalLabKey key(std::uint64_t nonce) { return {"SESSION_H4R_OPERATOR",3,nonce}; }
static DraftJournalLabDecision preview() {
  return {200,"{\"ok\":true}","{\"mission\":\"FULL_PREVIEW\",\"points\":[[0,0],[1,1]]}","PREVIEW"};
}
static DraftJournalLabDecision ready() {
  return {200,"{\"ok\":true}","{\"mission\":\"FULL_READY\",\"points\":[[0,0],[1,1]]}","READY"};
}
static std::uint64_t nonce_on_disk(const fs::path& path) {
  sqlite3* db=nullptr;
  assert(sqlite3_open_v2(path.string().c_str(),&db,SQLITE_OPEN_READWRITE,nullptr)==SQLITE_OK);
  sqlite3_stmt* st=nullptr;
  assert(sqlite3_prepare_v2(db,"SELECT nonce FROM replay_fence WHERE singleton=1",-1,&st,nullptr)==SQLITE_OK);
  assert(sqlite3_step(st)==SQLITE_ROW);
  const auto result=static_cast<std::uint64_t>(sqlite3_column_int64(st,0));
  sqlite3_finalize(st);
  assert(sqlite3_close_v2(db)==SQLITE_OK);
  return result;
}

static void nominal() {
  TempJournalDb tmp("nominal"); provision(tmp);
  NativeDraftSqliteMutationJournalLab journal(tmp.path.string());
  bool pending=false;
  assert(journal.has_pending(pending) && !pending);
  assert(journal.begin(proposal(1,1)));
  assert(nonce_on_disk(tmp.path)==1);
  assert(journal.has_pending(pending) && pending);
  assert(!journal.begin(proposal(2,2,"/api/v1/mission/update")));
  assert(nonce_on_disk(tmp.path)==1);
  DraftJournalLabEntry e;
  assert(journal.inspect(key(1),e) && e.status=="PENDING" && e.sequence==1);
  assert(!journal.conclude(key(1),ready())); // UPLOAD cannot become READY
  assert(journal.conclude(key(1),preview()));
  assert(!journal.conclude(key(1),preview())); // no second completion
  assert(journal.inspect(key(1),e) && e.status=="COMMITTED" &&
         e.decision.stage=="PREVIEW");
  assert(!journal.begin(proposal(1,2))); // replay nonce
  assert(!journal.begin(proposal(2,1,"/api/v1/mission/update"))); // duplicate sequence
  assert(nonce_on_disk(tmp.path)==1);
  assert(journal.begin(proposal(2,2,"/api/v1/mission/update")));
  const DraftJournalLabDecision denied{422,"{\"ok\":false}","",""};
  assert(journal.conclude(key(2),denied));
  assert(journal.inspect(key(2),e) && e.status=="DENIED" && e.decision.http_status==422);
  assert(journal.begin(proposal(3,3,"/api/v1/mission/load")));
  assert(journal.conclude(key(3),ready()));
  assert(journal.inspect(key(3),e) && e.status=="COMMITTED" && e.decision.stage=="READY");
  assert(journal.has_pending(pending) && !pending);
  assert(nonce_on_disk(tmp.path)==3);
}

static void recovery() {
  TempJournalDb tmp("recovery"); provision(tmp);
  {
    NativeDraftSqliteMutationJournalLab journal(tmp.path.string());
    assert(journal.begin(proposal(1,1)));
    // Deliberately crash/close before hypothetical backend reply is journaled.
  }
  {
    NativeDraftSqliteMutationJournalLab journal(tmp.path.string());
    bool pending=false;
    assert(journal.has_pending(pending) && pending);
    DraftJournalLabEntry e;
    assert(journal.inspect(key(1),e) && e.status=="PENDING");
    assert(e.canonical_request_json=="{\"sequence\":1}");
    assert(!journal.begin(proposal(1,1))); // never auto retry
    assert(!journal.begin(proposal(2,2))); // unresolved pending blocks new mutation
    assert(!journal.inspect(key(2),e));
  }
  // A separate normal case: completed record and snapshot survives restart.
  TempJournalDb done("completed"); provision(done);
  {
    NativeDraftSqliteMutationJournalLab journal(done.path.string());
    assert(journal.begin(proposal(1,1)));
    assert(journal.conclude(key(1),preview()));
  }
  {
    NativeDraftSqliteMutationJournalLab journal(done.path.string());
    DraftJournalLabEntry e;
    assert(journal.inspect(key(1),e) && e.status=="COMMITTED" &&
           e.decision.complete_snapshot_json==preview().complete_snapshot_json);
    bool pending=true;
    assert(journal.has_pending(pending) && !pending);
    assert(!journal.begin(proposal(1,1)));
    assert(journal.begin(proposal(2,2,"/api/v1/mission/update")));
  }
}

static void faults() {
  TempJournalDb tmp("faults");
  bool threw=false;
  try { NativeDraftSqliteMutationJournalLab absent(tmp.path.string()); }
  catch(const std::exception&) { threw=true; }
  assert(threw && !fs::exists(tmp.path));
  assert(!NativeDraftSqliteMutationJournalLab::provision_schema_for_existing_store(tmp.path.string()));
  offline_preprovision(tmp.path);
  threw=false;
  try { NativeDraftSqliteMutationJournalLab unschematized(tmp.path.string()); }
  catch(const std::exception&) { threw=true; }
  assert(threw);
  assert(NativeDraftSqliteMutationJournalLab::provision_schema_for_existing_store(tmp.path.string()));
  NativeDraftSqliteMutationJournalLab journal(tmp.path.string());
  auto p=proposal(1,1);
  p.operator_session_id="FORGED";
  assert(!journal.begin(p));
  p=proposal(1,1);p.session_epoch=4;
  assert(!journal.begin(p));
  p=proposal(1,1);p.route="/api/v1/mission/start";
  assert(!journal.begin(p));
  p=proposal(1,1);p.canonical_request_json="not json";
  assert(!journal.begin(p));
  p=proposal(1,1);p.received_sec=-3.0;
  assert(!journal.begin(p));
  assert(nonce_on_disk(tmp.path)==0);
  assert(journal.begin(proposal(1,1)));
  auto bad=preview();bad.stage="STARTED";
  assert(!journal.conclude(key(1),bad));
  bad=preview();bad.complete_snapshot_json="";
  assert(!journal.conclude(key(1),bad));
  bad=preview();bad.http_status=503;
  assert(!journal.conclude(key(1),bad));
  assert(!journal.conclude({"FORGED",3,1},preview()));
  assert(journal.conclude(key(1),preview()));
  p=proposal(2,2);p.received_sec=1.0; // rollback against existing clock
  assert(!journal.begin(p) && nonce_on_disk(tmp.path)==1);
  // No implicit schema repair on a new process after metadata tampering.
  {
    sqlite3* db=nullptr;
    assert(sqlite3_open_v2(tmp.path.string().c_str(),&db,SQLITE_OPEN_READWRITE,nullptr)==SQLITE_OK);
    assert(sqlite3_exec(db,"UPDATE dyx_journal_meta SET version=0",nullptr,nullptr,nullptr)!=SQLITE_OK);
    assert(sqlite3_exec(db,"DELETE FROM dyx_journal_meta",nullptr,nullptr,nullptr)==SQLITE_OK);
    sqlite3_close_v2(db);
  }
  threw=false;
  try { NativeDraftSqliteMutationJournalLab bad_schema(tmp.path.string()); }
  catch(const std::exception&) { threw=true; }
  assert(threw);
}

static void concurrent() {
  TempJournalDb tmp("concurrent"); provision(tmp);
  constexpr int N=8;
  std::vector<std::unique_ptr<NativeDraftSqliteMutationJournalLab>> stores;
  for(int i=0;i<N;++i) stores.emplace_back(
      std::make_unique<NativeDraftSqliteMutationJournalLab>(tmp.path.string()));
  std::vector<std::thread> threads;
  bool successes[N]{};
  for(int i=0;i<N;++i) threads.emplace_back([&,i] {
    successes[i]=stores[i]->begin(proposal(1,1));
  });
  for(auto& t:threads) t.join();
  int count=0;
  for(bool ok:successes) count+=ok?1:0;
  assert(count==1 && nonce_on_disk(tmp.path)==1);
  bool pending=false;
  assert(stores[0]->has_pending(pending) && pending);
}

int main(int argc,char** argv) {
  assert(argc==2);
  const std::string mode=argv[1];
  if(mode=="nominal") nominal();
  else if(mode=="recovery") recovery();
  else if(mode=="faults") faults();
  else if(mode=="concurrent") concurrent();
  else return 2;
  std::cout<<"PASS H4Z journal "<<mode<<'\n';
}
