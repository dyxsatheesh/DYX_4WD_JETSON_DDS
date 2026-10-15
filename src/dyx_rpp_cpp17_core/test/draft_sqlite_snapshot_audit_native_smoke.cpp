// DYX Autonomous H5C: offline saved mission snapshot consistency audit.
// Uses the actual journal/backend adapter under repository tests. No IO to PX4.
#include "backend_upload_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_snapshot_audit_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_sqlite_store_lab.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
namespace fs=std::filesystem;

struct H5cDb final {
  fs::path dir,path;
  explicit H5cDb(const char* label) {
    dir=fs::temp_directory_path()/(std::string("dyx_h5c_")+label+"_"+
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    assert(fs::create_directory(dir));
    path=dir/"store.sqlite";
    assert(NativeDraftSqliteStoreLab::provision_new(path.string(),
        {"SESSION_H4E_OPERATOR",3,0,0.0,1}));
    assert(NativeDraftSqliteMutationJournalLab::provision_schema_for_existing_store(path.string()));
  }
  ~H5cDb() { std::error_code ec; fs::remove_all(dir,ec); }
};
static DraftJournalLabProposal proposal(const BackendMissionUploadRequest& r,std::uint64_t nonce) {
  DraftJournalLabProposal p;
  p.operator_session_id=r.principal.operator_session_id;
  p.session_epoch=3;p.ingress_nonce=nonce;p.received_sec=r.host_received_sec;
  p.mission_run_id=r.mission_run_id;p.sequence=r.sequence;p.route=r.path;
  assert(journaled_upload_lab_detail::json_request(r,p.canonical_request_json));
  return p;
}
static DraftReconciliationLabIdentity identity() {
  return {true,true,true,"SESSION_H4E_OPERATOR",3};
}
static DraftJournalLabKey key(std::uint64_t nonce) {
  return {"SESSION_H4E_OPERATOR",3,nonce};
}
static void mutate(const fs::path& path,const char* sql) {
  sqlite3* db=nullptr;
  assert(sqlite3_open_v2(path.string().c_str(),&db,
                        SQLITE_OPEN_READWRITE,nullptr)==SQLITE_OK);
  assert(sqlite3_exec(db,sql,nullptr,nullptr,nullptr)==SQLITE_OK);
  assert(sqlite3_changes(db)==1);
  assert(sqlite3_close_v2(db)==SQLITE_OK);
}
static void run_preview() {
  H5cDb db("preview");
  NativeDraftSqliteMutationJournalLab journal(db.path.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(journal,backend);
  const auto r=upload_req(kBackendMissionUploadPath,1,10);
  const auto p=proposal(r,1);
  assert(writer.submit(p,r).journal_completed);
  NativeOfflineDraftSnapshotAuditLab audit(journal);
  auto result=audit.audit(identity(),key(1),p.canonical_request_json);
  assert(result.state==DraftSnapshotAuditLabState::consistent_preview);
  assert(result.snapshot_consistency_verified && result.revision==1);
  assert(result.marking_count==2 && result.navigation_count>=2);
  assert(valid_path_signature(result.snapshot_signature));
  assert(!result.can_restore && !result.can_start_or_resume &&
         !result.hardware_command_emitted && !gateway.emits_hardware_commands());
  // Disk reopen is read-only. Neither reconstruction nor mission start occurs.
  NativeDraftSqliteMutationJournalLab reopened(db.path.string());
  NativeOfflineDraftSnapshotAuditLab second(reopened);
  const auto recovered=second.audit(identity(),key(1),p.canonical_request_json);
  assert(recovered.state==DraftSnapshotAuditLabState::consistent_preview);
  assert(!recovered.can_restore && !recovered.can_start_or_resume);
}
static void run_ready() {
  H5cDb db("ready");
  NativeDraftSqliteMutationJournalLab journal(db.path.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(journal,backend);
  auto r=upload_req(kBackendMissionUploadPath,1,10);
  auto p=proposal(r,1);
  const auto uploaded=writer.submit(p,r);
  assert(uploaded.journal_completed && uploaded.backend_reply.http_status==200);
  const auto load=load_req(uploaded.backend_reply,2,11);
  const auto p2=proposal(load,2);
  const auto loaded=writer.submit(p2,load);
  assert(loaded.journal_completed && loaded.backend_reply.loaded);
  NativeOfflineDraftSnapshotAuditLab audit(journal);
  const auto result=audit.audit(identity(),key(2),p2.canonical_request_json);
  assert(result.state==DraftSnapshotAuditLabState::consistent_ready);
  assert(result.snapshot_consistency_verified && result.revision==1);
  assert(!result.can_restore && !result.can_start_or_resume);
  assert(gateway.mission().stage()==MissionStage::ready);
  assert(!gateway.emits_hardware_commands());
}
static void run_tamper() {
  H5cDb db("tamper");
  NativeDraftSqliteMutationJournalLab journal(db.path.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(journal,backend);
  const auto r=upload_req(kBackendMissionUploadPath,1,10);
  const auto p=proposal(r,1);
  assert(writer.submit(p,r).journal_completed);
  NativeOfflineDraftSnapshotAuditLab audit(journal);
  assert(audit.audit(identity(),key(1),p.canonical_request_json).snapshot_consistency_verified);
  mutate(db.path,"UPDATE dyx_mission_mutations SET "
       "snapshot_json=REPLACE(snapshot_json, '[2,0]', '[3,0]') WHERE nonce=1");
  const auto modified=audit.audit(identity(),key(1),p.canonical_request_json);
  assert(modified.state==DraftSnapshotAuditLabState::inconsistent_snapshot);
  assert(!modified.snapshot_consistency_verified && !modified.can_restore);
  // Even if content were structurally valid, loaded/route mismatch is denied.
  mutate(db.path,"UPDATE dyx_mission_mutations SET "
       "snapshot_json=REPLACE(snapshot_json, '\"loaded\":false', '\"loaded\":true') WHERE nonce=1");
  assert(audit.audit(identity(),key(1),p.canonical_request_json).state==
         DraftSnapshotAuditLabState::inconsistent_snapshot);
}
static void run_security() {
  H5cDb db("security");
  NativeDraftSqliteMutationJournalLab journal(db.path.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(journal,backend);
  const auto r=upload_req(kBackendMissionUploadPath,1,10);
  const auto p=proposal(r,1);
  assert(writer.submit(p,r).journal_completed);
  NativeOfflineDraftSnapshotAuditLab audit(journal);
  auto forged=identity();forged.authenticated=false;
  assert(audit.audit(forged,key(1),p.canonical_request_json).state==
         DraftSnapshotAuditLabState::access_denied);
  forged=identity();forged.session_epoch=4;
  assert(audit.audit(forged,key(1),p.canonical_request_json).state==
         DraftSnapshotAuditLabState::access_denied);
  assert(!audit.audit(identity(),key(1),"{\"wrong\":true}").snapshot_consistency_verified);
  assert(!audit.audit(identity(),key(2),p.canonical_request_json).snapshot_consistency_verified);
  // A separate committed PENDING is not a successful saved snapshot.
  H5cDb pending_db("pending");
  NativeDraftSqliteMutationJournalLab pending_journal(pending_db.path.string());
  assert(pending_journal.begin(p));
  NativeOfflineDraftSnapshotAuditLab pending_audit(pending_journal);
  assert(pending_audit.audit(identity(),key(1),p.canonical_request_json).state==
         DraftSnapshotAuditLabState::pending_review);
}
int main(int argc,char**argv) {
  assert(argc==2);
  const std::string mode=argv[1];
  if(mode=="preview") run_preview();
  else if(mode=="ready") run_ready();
  else if(mode=="tamper") run_tamper();
  else if(mode=="security") run_security();
  else return 2;
  std::cout<<"PASS H5C snapshot audit "<<mode<<'\n';
}
