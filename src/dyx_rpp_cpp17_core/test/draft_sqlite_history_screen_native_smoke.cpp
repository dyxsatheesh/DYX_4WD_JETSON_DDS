// DYX Autonomous H5D. OFFLINE full-history SQLite screening tests.
#include "backend_upload_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_history_screen_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_sqlite_store_lab.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
namespace fs=std::filesystem;
using namespace dyx::rpp;
struct LabDb {
  fs::path dir,file;
  explicit LabDb(const std::string& label) {
    const auto now=std::chrono::steady_clock::now().time_since_epoch().count();
    dir=fs::temp_directory_path()/("dyx_h5d_"+label+"_"+std::to_string(now));
    assert(fs::create_directory(dir));
    file=dir/"mission.sqlite";
    assert(NativeDraftSqliteStoreLab::provision_new(file.string(),
        {"SESSION_H4E_OPERATOR",3,0,0.0,1}));
    assert(NativeDraftSqliteMutationJournalLab::provision_schema_for_existing_store(file.string()));
  }
  ~LabDb(){std::error_code ec;fs::remove_all(dir,ec);}
};
static DraftReconciliationLabIdentity allowed() {
  return {true,true,true,"SESSION_H4E_OPERATOR",3};
}
static DraftJournalLabProposal proposal(const BackendMissionUploadRequest& r,std::uint64_t nonce) {
  DraftJournalLabProposal p;
  p.operator_session_id=r.principal.operator_session_id;
  p.session_epoch=3;p.ingress_nonce=nonce;
  p.received_sec=r.host_received_sec;
  p.mission_run_id=r.mission_run_id;p.sequence=r.sequence;p.route=r.path;
  assert(journaled_upload_lab_detail::json_request(r,p.canonical_request_json));
  return p;
}
static void modify(const fs::path& dbpath,const char* sql) {
  sqlite3* db=nullptr;
  assert(sqlite3_open_v2(dbpath.string().c_str(),&db,SQLITE_OPEN_READWRITE,nullptr)==SQLITE_OK);
  assert(sqlite3_exec(db,sql,nullptr,nullptr,nullptr)==SQLITE_OK);
  assert(sqlite3_changes(db)==1);
  assert(sqlite3_close_v2(db)==SQLITE_OK);
}
static void nominal() {
  LabDb db("nominal");
  NativeOfflineDraftHistoryScreenLab screen(db.file.string());
  assert(screen.inspect(allowed()).state==DraftHistoryScreenLabState::no_saved_mutation);
  NativeDraftSqliteMutationJournalLab j(db.file.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(j,backend);
  auto up=upload_req(kBackendMissionUploadPath,1,10);
  const auto first=writer.submit(proposal(up,1),up);
  assert(first.journal_completed && first.backend_reply.http_status==200);
  auto first_view=screen.inspect(allowed());
  assert(first_view.state==DraftHistoryScreenLabState::consistent_preview_history &&
         first_view.committed_count==1 && first_view.revision==1 &&
         first_view.history_consistency_verified);
  auto upd=update_req(first.backend_reply,2,11);
  upd.marking_points[1].x=3;
  const auto second=writer.submit(proposal(upd,2),upd);
  assert(second.journal_completed && second.backend_reply.revision==2);
  auto view=screen.inspect(allowed());
  assert(view.state==DraftHistoryScreenLabState::consistent_preview_history &&
         view.committed_count==2 && view.revision==2 &&
         view.signature==second.backend_reply.signature &&
         view.marking_count==2 && view.history_consistency_verified &&
         !view.can_restore && !view.can_start_or_resume &&
         !view.hardware_command_emitted && !gateway.emits_hardware_commands());
  // Reopen read-only screening after process loss: still no gateway restore.
  NativeOfflineDraftHistoryScreenLab reopened(db.file.string());
  assert(reopened.inspect(allowed()).state==DraftHistoryScreenLabState::consistent_preview_history);
}
static void ready() {
  LabDb db("ready");
  NativeDraftSqliteMutationJournalLab j(db.file.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(j,backend);
  const auto up=upload_req(kBackendMissionUploadPath,1,10);
  const auto first=writer.submit(proposal(up,1),up);
  assert(first.journal_completed);
  const auto load=load_req(first.backend_reply,2,11);
  const auto committed=writer.submit(proposal(load,2),load);
  assert(committed.journal_completed && committed.backend_reply.loaded);
  NativeOfflineDraftHistoryScreenLab screen(db.file.string());
  const auto result=screen.inspect(allowed());
  assert(result.state==DraftHistoryScreenLabState::ready_operator_review &&
         result.revision==1 && result.committed_count==2 &&
         result.history_consistency_verified && !result.can_start_or_resume &&
         !result.can_restore && !gateway.emits_hardware_commands());
}
static void pending() {
  LabDb db("pending");
  NativeDraftSqliteMutationJournalLab j(db.file.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(j,backend);
  auto up=upload_req(kBackendMissionUploadPath,1,10);
  auto first=writer.submit(proposal(up,1),up);
  assert(first.journal_completed);
  auto upd=update_req(first.backend_reply,2,11);
  assert(j.begin(proposal(upd,2))); // simulate power loss BEFORE conclude
  NativeOfflineDraftHistoryScreenLab screen(db.file.string());
  auto result=screen.inspect(allowed());
  assert(result.state==DraftHistoryScreenLabState::pending_operator_review &&
         !result.history_consistency_verified && !result.can_restore);
  assert(!gateway.emits_hardware_commands());
}
static void tamper() {
  LabDb db("tamper");
  NativeDraftSqliteMutationJournalLab j(db.file.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(j,backend);
  auto up=upload_req(kBackendMissionUploadPath,1,10);
  auto first=writer.submit(proposal(up,1),up);
  assert(first.journal_completed);
  auto upd=update_req(first.backend_reply,2,11);
  upd.marking_points[1].x=3;
  const auto second=writer.submit(proposal(upd,2),upd);
  assert(second.journal_completed);
  NativeOfflineDraftHistoryScreenLab screen(db.file.string());
  assert(screen.inspect(allowed()).history_consistency_verified);
  // Rewrite an otherwise well-signed second snapshot revision to break chain.
  modify(db.file,"UPDATE dyx_mission_mutations SET "
         "snapshot_json=REPLACE(snapshot_json,'\"revision\":2','\"revision\":1') WHERE nonce=2");
  auto result=screen.inspect(allowed());
  assert(result.state==DraftHistoryScreenLabState::inconsistent_history &&
         !result.history_consistency_verified);
  // Even without geometry tamper, request's expected revision must match.
  modify(db.file,"UPDATE dyx_mission_mutations SET "
         "snapshot_json=REPLACE(snapshot_json,'\"revision\":1','\"revision\":2') WHERE nonce=2");
  modify(db.file,"UPDATE dyx_mission_mutations SET "
         "request_json=REPLACE(request_json,'\"expected_revision\":1','\"expected_revision\":2') WHERE nonce=2");
  assert(screen.inspect(allowed()).state==DraftHistoryScreenLabState::inconsistent_history);
  assert(!gateway.emits_hardware_commands());
}
static void security() {
  LabDb db("security");
  NativeOfflineDraftHistoryScreenLab screen(db.file.string());
  auto bad=allowed();bad.authenticated=false;
  assert(screen.inspect(bad).state==DraftHistoryScreenLabState::access_denied);
  bad=allowed();bad.session_epoch=4;
  assert(screen.inspect(bad).state==DraftHistoryScreenLabState::access_denied);
  bad=allowed();bad.operator_session_id="FORGED";
  assert(screen.inspect(bad).state==DraftHistoryScreenLabState::access_denied);
  assert(NativeOfflineDraftHistoryScreenLab("/tmp/dyx_h5d_nonexistent_db_dont_create.sqlite")
       .inspect(allowed()).state==DraftHistoryScreenLabState::storage_error);
  assert(screen.inspect(allowed()).state==DraftHistoryScreenLabState::no_saved_mutation);
  // A rejected mutation is not automatically a clean recoverable chain.
  NativeDraftSqliteMutationJournalLab j(db.file.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(j,backend);
  auto up=upload_req(kBackendMissionUploadPath,1,10);
  auto first=writer.submit(proposal(up,1),up);
  assert(first.journal_completed);
  auto upd=update_req(first.backend_reply,2,11);
  upd.expected_revision=999; // backend denies AFTER journal reservation
  auto denied=writer.submit(proposal(upd,2),upd);
  assert(denied.journal_completed && denied.backend_reply.http_status>=400);
  assert(screen.inspect(allowed()).state==DraftHistoryScreenLabState::denial_operator_review);
}
int main(int argc,char**argv){
  assert(argc==2);
  const std::string mode=argv[1];
  if(mode=="nominal")nominal();
  else if(mode=="ready")ready();
  else if(mode=="pending")pending();
  else if(mode=="tamper")tamper();
  else if(mode=="security")security();
  else return 2;
  std::cout<<"PASS H5D read-only history screen "<<mode<<'\n';
}
