// DYX Autonomous H5H: offline, write-blocked epoch-quarantine smoke tests.
#include "backend_upload_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_epoch_quarantine_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_sqlite_store_lab.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
namespace fs=std::filesystem;
using namespace dyx::rpp;
struct LocalDb {
  fs::path dir,file;
  explicit LocalDb(const std::string& name) {
    const auto n=std::chrono::steady_clock::now().time_since_epoch().count();
    dir=fs::temp_directory_path()/("dyx_h5h_"+name+"_"+std::to_string(n));
    assert(fs::create_directory(dir));
    file=dir/"mission.sqlite";
    assert(NativeDraftSqliteStoreLab::provision_new(file.string(),
       {"SESSION_H4E_OPERATOR",3,0,0.0,1}));
    assert(NativeDraftSqliteMutationJournalLab::provision_schema_for_existing_store(file.string()));
  }
  ~LocalDb(){std::error_code e;fs::remove_all(dir,e);}
};
static DraftRecoveryPreflightLabRequest trusted() {
  DraftRecoveryPreflightLabRequest r;
  r.saved_identity={true,true,true,"SESSION_H4E_OPERATOR",3};
  r.proposed_new_epoch=4;
  r.explicit_operator_review=true;
  r.independently_disarmed=true;
  r.independently_stationary=true;
  r.independently_no_active_mission=true;
  return r;
}
static DraftJournalLabProposal proposal(const BackendMissionUploadRequest& r,
                                        std::uint64_t nonce,std::uint64_t epoch=3) {
  DraftJournalLabProposal p;
  p.operator_session_id=r.principal.operator_session_id;
  p.session_epoch=epoch;p.ingress_nonce=nonce;p.received_sec=r.host_received_sec;
  p.mission_run_id=r.mission_run_id;p.sequence=r.sequence;p.route=r.path;
  assert(journaled_upload_lab_detail::json_request(r,p.canonical_request_json));
  return p;
}
static BackendMissionUploadResponse upload(NativeDraftSqliteMutationJournalLab& journal,
                                          NativeBackendMissionUpload& backend) {
  NativeOfflineDraftJournaledUploadLab writer(journal,backend);
  auto req=upload_req(kBackendMissionUploadPath,1,10);
  const auto result=writer.submit(proposal(req,1),req);
  assert(result.journal_completed && result.backend_reply.http_status==200);
  return result.backend_reply;
}
static void seal(const fs::path& file) {
  NativeOfflineDraftPreviewCheckpointLab c(file.string());
  assert(c.seal_preview(trusted().saved_identity).state==
       DraftPreviewCheckpointLabState::sealed_preview);
}
static DraftRestartLabRecord fence(const fs::path& f) {
  NativeDraftSqliteStoreLab store(f.string());
  DraftRestartLabRecord out;
  assert(store.load_existing(out));
  return out;
}
static int barrier_count(const fs::path& file) {
  auto* db=draft_journal_lab_detail::open_existing(file.string());
  draft_journal_lab_detail::Statement st(db,
      "SELECT count(*) FROM dyx_mission_recovery_epoch_barrier");
  assert(st && sqlite3_step(st.st)==SQLITE_ROW);
  const int count=sqlite3_column_int(st.st,0);
  sqlite3_finalize(st.st); st.st=nullptr;
  sqlite3_close_v2(db);
  return count;
}
static void nominal() {
  LocalDb db("nominal");
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  NativeMissionCommandGateway previous;
  NativeBackendMissionUpload previous_backend(previous);
  upload(journal,previous_backend);seal(db.file);
  NativeMissionCommandGateway fresh;
  NativeBackendMissionUpload empty(fresh);
  auto result=NativeOfflineDraftEpochQuarantineLab(db.file.string()).quarantine(trusted(),fresh,empty);
  assert(result.state==DraftEpochQuarantineLabState::quarantine_committed &&
         result.saved_epoch==3 && result.quarantined_epoch==4 &&
         result.last_nonce==1 && result.last_sequence==1 &&
         result.fence_generation==3 && !result.signature.empty() &&
         !result.can_post && !result.can_load && !result.can_start_or_resume &&
         !result.hardware_command_emitted && !fresh.emits_hardware_commands());
  auto f=fence(db.file);
  assert(f.operator_session_id=="SESSION_H4E_OPERATOR" &&
         f.session_epoch==4 && f.last_ingress_nonce==1 &&
         f.generation==3 && f.last_received_sec==10.0 && barrier_count(db.file)==1);
  assert(!empty.has_draft() && !fresh.has_active_mission());
}
static void restart() {
  LocalDb db("restart");
  { NativeDraftSqliteMutationJournalLab j(db.file.string());
    NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
    upload(j,b);seal(db.file); }
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  NativeOfflineDraftEpochQuarantineLab q(db.file.string());
  assert(q.quarantine(trusted(),g,b).state==DraftEpochQuarantineLabState::quarantine_committed);
  NativeDraftSqliteMutationJournalLab reopened(db.file.string());
  bool exists=false;assert(reopened.has_history(exists) && exists);
  const auto old=upload_req(kBackendMissionUploadPath,2,11);
  assert(!reopened.begin(proposal(old,2,3)));
  // Even a request forged with the newly rotated epoch remains quarantined.
  assert(!reopened.begin(proposal(old,2,4)));
  assert(q.quarantine(trusted(),g,b).state==DraftEpochQuarantineLabState::already_quarantined);
  assert(barrier_count(db.file)==1);
}
static void authorization() {
  LocalDb db("authorization");
  NativeDraftSqliteMutationJournalLab j(db.file.string());
  NativeMissionCommandGateway existing;NativeBackendMissionUpload backend(existing);
  upload(j,backend);seal(db.file);
  NativeMissionCommandGateway g;NativeBackendMissionUpload empty(g);
  NativeOfflineDraftEpochQuarantineLab q(db.file.string());
  auto r=trusted();r.explicit_operator_review=false;
  assert(q.quarantine(r,g,empty).state==DraftEpochQuarantineLabState::invalid_request);
  r=trusted();r.independently_disarmed=false;
  assert(q.quarantine(r,g,empty).state==DraftEpochQuarantineLabState::invalid_request);
  r=trusted();r.independently_stationary=false;
  assert(q.quarantine(r,g,empty).state==DraftEpochQuarantineLabState::invalid_request);
  r=trusted();r.independently_no_active_mission=false;
  assert(q.quarantine(r,g,empty).state==DraftEpochQuarantineLabState::invalid_request);
  r=trusted();r.proposed_new_epoch=3;
  assert(q.quarantine(r,g,empty).state==DraftEpochQuarantineLabState::invalid_request);
  r=trusted();r.saved_identity.authenticated=false;
  assert(q.quarantine(r,g,empty).state==DraftEpochQuarantineLabState::invalid_request);
  r=trusted();r.saved_identity.operator_session_id="FORGED";
  assert(q.quarantine(r,g,empty).state==DraftEpochQuarantineLabState::inconsistent_fence);
  assert(q.quarantine(trusted(),existing,backend).state==
       DraftEpochQuarantineLabState::active_backend_or_gateway);
  assert(fence(db.file).session_epoch==3 && barrier_count(db.file)==0);
}
static void stale() {
  LocalDb db("stale");
  NativeDraftSqliteMutationJournalLab j(db.file.string());
  NativeMissionCommandGateway g;NativeBackendMissionUpload backend(g);
  NativeOfflineDraftJournaledUploadLab w(j,backend);
  auto initial=upload_req(kBackendMissionUploadPath,1,10);
  auto initial_result=w.submit(proposal(initial,1),initial);
  assert(initial_result.journal_completed && initial_result.backend_reply.http_status==200);
  auto first=initial_result.backend_reply;
  seal(db.file);
  auto req=update_req(first,2,11);
  assert(w.submit(proposal(req,2),req).journal_completed);
  NativeMissionCommandGateway fresh;NativeBackendMissionUpload empty(fresh);
  const auto result=NativeOfflineDraftEpochQuarantineLab(db.file.string()).quarantine(trusted(),fresh,empty);
  assert(result.state==DraftEpochQuarantineLabState::preflight_rejected &&
         fence(db.file).session_epoch==3 && barrier_count(db.file)==0);
}
static void pending() {
  LocalDb db("pending");
  NativeDraftSqliteMutationJournalLab j(db.file.string());
  NativeMissionCommandGateway g;NativeBackendMissionUpload backend(g);
  auto first=upload(j,backend);seal(db.file);
  auto req=update_req(first,2,11);
  assert(j.begin(proposal(req,2)));
  NativeMissionCommandGateway fresh;NativeBackendMissionUpload empty(fresh);
  assert(NativeOfflineDraftEpochQuarantineLab(db.file.string()).quarantine(trusted(),fresh,empty).state==
         DraftEpochQuarantineLabState::preflight_rejected);
  assert(fence(db.file).session_epoch==3 && barrier_count(db.file)==0);
}
static void ready() {
  LocalDb db("ready");
  NativeDraftSqliteMutationJournalLab j(db.file.string());
  NativeMissionCommandGateway g;NativeBackendMissionUpload backend(g);
  NativeOfflineDraftJournaledUploadLab w(j,backend);
  auto initial=upload_req(kBackendMissionUploadPath,1,10);
  auto initial_result=w.submit(proposal(initial,1),initial);
  assert(initial_result.journal_completed && initial_result.backend_reply.http_status==200);
  auto first=initial_result.backend_reply;
  seal(db.file);
  auto req=load_req(first,2,11);
  assert(w.submit(proposal(req,2),req).journal_completed);
  NativeMissionCommandGateway fresh;NativeBackendMissionUpload empty(fresh);
  assert(NativeOfflineDraftEpochQuarantineLab(db.file.string()).quarantine(trusted(),fresh,empty).state==
         DraftEpochQuarantineLabState::preflight_rejected);
  assert(fence(db.file).session_epoch==3 && barrier_count(db.file)==0);
}
static void failure() {
  LocalDb db("failure");
  NativeDraftSqliteMutationJournalLab j(db.file.string());
  NativeMissionCommandGateway g;NativeBackendMissionUpload backend(g);
  upload(j,backend);seal(db.file);
  { // Deterministically abort the barrier insert inside BEGIN IMMEDIATE.
    sqlite3* sql=draft_journal_lab_detail::open_existing(db.file.string());
    assert(draft_journal_lab_detail::exec(sql,
      "CREATE TRIGGER fail_h5h BEFORE INSERT ON dyx_mission_recovery_epoch_barrier "
      "BEGIN SELECT RAISE(ABORT,'injected'); END"));
    sqlite3_close_v2(sql);
  }
  NativeMissionCommandGateway fresh;NativeBackendMissionUpload empty(fresh);
  assert(NativeOfflineDraftEpochQuarantineLab(db.file.string()).quarantine(trusted(),fresh,empty).state==
         DraftEpochQuarantineLabState::persistence_fault);
  assert(fence(db.file).session_epoch==3 && barrier_count(db.file)==0);
}
static void schema() {
  LocalDb db("schema");
  sqlite3* sql=draft_journal_lab_detail::open_existing(db.file.string());
  assert(draft_journal_lab_detail::exec(sql,"DROP TABLE dyx_mission_recovery_epoch_barrier"));
  sqlite3_close_v2(sql);
  bool rejected=false;
  try { NativeDraftSqliteMutationJournalLab j(db.file.string()); }
  catch(const std::exception&) {rejected=true;}
  assert(rejected);
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  assert(NativeOfflineDraftEpochQuarantineLab(db.file.string()).quarantine(trusted(),g,b).state==
         DraftEpochQuarantineLabState::storage_error);
  assert(fence(db.file).session_epoch==3);
}
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  const std::string mode=argv[1];
  if(mode=="nominal")nominal();
  else if(mode=="restart")restart();
  else if(mode=="authorization")authorization();
  else if(mode=="stale")stale();
  else if(mode=="pending")pending();
  else if(mode=="ready")ready();
  else if(mode=="failure")failure();
  else if(mode=="schema")schema();
  else return 2;
  std::cout<<"PASS H5H "<<mode<<"\n";
}
