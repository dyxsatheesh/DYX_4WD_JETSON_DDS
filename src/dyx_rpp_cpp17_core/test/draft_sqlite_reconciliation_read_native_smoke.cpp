// Phase 2H5B. Read-only authenticated-evidence fixture, no Android/PX4 I/O.
#include "dyx_rpp_cpp17_core/backend_draft_reconciliation_read_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_sqlite_store_lab.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
using namespace dyx::rpp;
namespace fs=std::filesystem;

struct H5bDb final {
  fs::path dir,path;
  explicit H5bDb(const char* mode) {
    const auto n=std::chrono::steady_clock::now().time_since_epoch().count();
    dir=fs::temp_directory_path()/(std::string("dyx_h5b_")+mode+"_"+std::to_string(n));
    assert(fs::create_directory(dir));
    path=dir/"journal.sqlite";
    assert(NativeDraftSqliteStoreLab::provision_new(path.string(),
        {"SESSION_H5B_OPERATOR",5,0,0.0,1}));
    assert(NativeDraftSqliteMutationJournalLab::provision_schema_for_existing_store(path.string()));
  }
  ~H5bDb(){std::error_code ignored;fs::remove_all(dir,ignored);}
};
static const std::string request_json="{\"mission_run_id\":\"RUN_H5B_001\",\"sequence\":1}";
static DraftJournalLabProposal proposal(std::uint64_t nonce,std::uint64_t sequence) {
  return {"SESSION_H5B_OPERATOR",5,nonce,static_cast<double>(10+nonce),
          "RUN_H5B_001",sequence,"/api/v1/mission/upload",request_json};
}
static DraftJournalLabKey key(std::uint64_t nonce) {
  return {"SESSION_H5B_OPERATOR",5,nonce};
}
static DraftReconciliationLabIdentity trusted() {
  return {true,true,true,"SESSION_H5B_OPERATOR",5};
}
static void assert_never_authorizes(const DraftReconciliationLabReply& r) {
  assert(!r.is_verified_snapshot && !r.can_restore_mission &&
         !r.can_start_or_resume && !r.hardware_command_emitted);
}
static void preview() {
  H5bDb db("preview");
  NativeDraftSqliteMutationJournalLab j(db.path.string());
  assert(j.begin(proposal(1,1)));
  assert(j.conclude(key(1),{200,"{\"ok\":true}","{\"snapshot\":true}","PREVIEW"}));
  NativeOfflineDraftReconciliationReadLab read(j);
  auto r=read.inspect(trusted(),key(1),request_json);
  assert(r.http_status==200 && r.state==DraftReconciliationLabState::recorded_preview &&
         r.recorded_http_status==200 && r.sequence==1 && r.mission_run_id=="RUN_H5B_001");
  assert_never_authorizes(r);
  // Same result survives SQLite close/open without recovering backend memory.
  { NativeDraftSqliteMutationJournalLab reopen(db.path.string());
    NativeOfflineDraftReconciliationReadLab other(reopen);
    r=other.inspect(trusted(),key(1),request_json);
    assert(r.state==DraftReconciliationLabState::recorded_preview);
    assert_never_authorizes(r);
  }
}
static void pending() {
  H5bDb db("pending");
  NativeDraftSqliteMutationJournalLab j(db.path.string());
  assert(j.begin(proposal(1,1)));
  NativeOfflineDraftReconciliationReadLab read(j);
  auto r=read.inspect(trusted(),key(1),request_json);
  assert(r.http_status==202 && r.state==DraftReconciliationLabState::pending_operator_review &&
         r.recorded_http_status==0);
  assert_never_authorizes(r);
  assert(!j.begin(proposal(2,2)));
}
static void denied() {
  H5bDb db("denied");
  NativeDraftSqliteMutationJournalLab j(db.path.string());
  assert(j.begin(proposal(1,1)));
  assert(j.conclude(key(1),{422,"{\"error\":true}","",""}));
  NativeOfflineDraftReconciliationReadLab read(j);
  const auto r=read.inspect(trusted(),key(1),request_json);
  assert(r.http_status==200 && r.state==DraftReconciliationLabState::recorded_denial &&
         r.recorded_http_status==422);
  assert_never_authorizes(r);
}
static void binding() {
  H5bDb db("binding");
  NativeDraftSqliteMutationJournalLab j(db.path.string());
  assert(j.begin(proposal(1,1)));
  NativeOfflineDraftReconciliationReadLab read(j);
  auto other=trusted();other.operator_session_id="OTHER_USER";
  auto r=read.inspect(other,key(1),request_json);
  assert(r.http_status==403 && r.mission_run_id.empty());assert_never_authorizes(r);
  other=trusted();other.fresh_session_evidence=false;
  assert(read.inspect(other,key(1),request_json).http_status==403);
  other=trusted();other.can_read_mission=false;
  assert(read.inspect(other,key(1),request_json).http_status==403);
  other=trusted();other.session_epoch=6;
  assert(read.inspect(other,key(1),request_json).http_status==403);
  r=read.inspect(trusted(),key(1),"{\"sequence\":99}");
  assert(r.http_status==409 && r.mission_run_id.empty());assert_never_authorizes(r);
  r=read.inspect(trusted(),key(2),request_json);
  assert(r.http_status==503 && r.mission_run_id.empty());assert_never_authorizes(r);
}
int main(int argc,char** argv){
  assert(argc==2);
  const std::string mode=argv[1];
  if(mode=="preview")preview();
  else if(mode=="pending")pending();
  else if(mode=="denied")denied();
  else if(mode=="binding")binding();
  else return 2;
  std::cout<<"PASS H5B read-only "<<mode<<'\n';
}
