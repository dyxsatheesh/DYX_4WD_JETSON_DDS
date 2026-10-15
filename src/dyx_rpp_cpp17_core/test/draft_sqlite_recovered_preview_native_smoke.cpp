// DYX Autonomous Phase 2H5G: detached, command-inert PREVIEW read model.
#include "backend_upload_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_recovered_preview_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_sqlite_store_lab.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
namespace fs=std::filesystem;
using namespace dyx::rpp;
struct LocalDb {
  fs::path dir,file;
  explicit LocalDb(const std::string& name) {
    const auto n=std::chrono::steady_clock::now().time_since_epoch().count();
    dir=fs::temp_directory_path()/("dyx_h5g_"+name+"_"+std::to_string(n));
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
                                        std::uint64_t nonce) {
  DraftJournalLabProposal p;
  p.operator_session_id=r.principal.operator_session_id;
  p.session_epoch=3;p.ingress_nonce=nonce;p.received_sec=r.host_received_sec;
  p.mission_run_id=r.mission_run_id;p.sequence=r.sequence;p.route=r.path;
  assert(journaled_upload_lab_detail::json_request(r,p.canonical_request_json));
  return p;
}
static BackendMissionUploadResponse persist_upload(const fs::path& path,
                                                  NativeBackendMissionUpload& backend,
                                                  MissionMode mode=MissionMode::automatic) {
  NativeDraftSqliteMutationJournalLab journal(path.string());
  NativeOfflineDraftJournaledUploadLab writer(journal,backend);
  auto up=upload_req(kBackendMissionUploadPath,1,10);
  up.mission_mode=mode;
  const auto result=writer.submit(proposal(up,1),up);
  assert(result.journal_completed && result.backend_reply.http_status==200);
  return result.backend_reply;
}
static void checkpoint(const fs::path& path) {
  NativeOfflineDraftPreviewCheckpointLab cp(path.string());
  assert(cp.seal_preview(trusted().saved_identity).state==
       DraftPreviewCheckpointLabState::sealed_preview);
}
static BackendMissionUploadRequest get_req() {
  auto r=upload_req(kBackendMissionDraftPath,0,0);
  r.method="GET";
  r.principal.owns_control_lease=false;
  r.marking_points.clear();
  return r;
}
static void nominal() {
  LocalDb db("nominal");
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  persist_upload(db.file,backend);
  checkpoint(db.file);
  NativeOfflineRecoveredDraftPreviewLab restored(db.file.string());
  const auto result=restored.reconstruct_for_review(trusted(),gateway);
  assert(result.state==DraftRecoveredPreviewLabState::preview_reconstructed &&
         result.revision==1 && result.saved_epoch==3 &&
         result.proposed_epoch==4 && result.last_nonce==1 && result.last_sequence==1 &&
         !result.can_restore_backend && !result.can_load &&
         !result.can_start_or_resume && !result.hardware_command_emitted);
  const auto view=restored.handle(get_req());
  assert(view.http_status==200 && view.code=="RECOVERED_PREVIEW_READ_ONLY" &&
         view.draft_ready && !view.loaded && view.preview_only &&
         view.signature==result.signature && !gateway.emits_hardware_commands());
  const auto snapshot=restored.review_snapshot(get_req());
  assert(snapshot && snapshot->marking_points.size()==2);
  assert(restored.review_mode(get_req())==MissionMode::automatic);
  assert(restored.reconstruct_for_review(trusted(),gateway).state==
         DraftRecoveredPreviewLabState::already_reconstructed);
}
static void restart() {
  LocalDb db("restart");
  {
    NativeMissionCommandGateway g;
    NativeBackendMissionUpload b(g);
    persist_upload(db.file,b);
    checkpoint(db.file);
  }
  NativeMissionCommandGateway fresh_gateway;
  NativeBackendMissionUpload fresh_backend(fresh_gateway);
  NativeOfflineRecoveredDraftPreviewLab recovered(db.file.string());
  assert(recovered.reconstruct_for_review(trusted(),fresh_gateway).state==
         DraftRecoveredPreviewLabState::preview_reconstructed);
  assert(recovered.handle(get_req()).http_status==200);
  assert(!fresh_backend.has_draft() && !fresh_gateway.has_active_mission() &&
         !fresh_gateway.emits_hardware_commands());
  auto copied=recovered.review_snapshot(get_req());
  assert(copied);
  copied->marking_points[0].x=9999.0;
  assert(recovered.review_snapshot(get_req())->marking_points[0].x!=9999.0);
}
static void requests() {
  LocalDb db("requests");
  NativeMissionCommandGateway g;
  NativeBackendMissionUpload b(g);
  persist_upload(db.file,b);checkpoint(db.file);
  NativeOfflineRecoveredDraftPreviewLab recovered(db.file.string());
  assert(recovered.reconstruct_for_review(trusted(),g).state==
         DraftRecoveredPreviewLabState::preview_reconstructed);
  for(const char* route : {kBackendMissionUploadPath,kBackendMissionUpdatePath,
                           kBackendMissionLoadPath}) {
    auto r=upload_req(route,100,100);
    r.trusted_px4_disarmed=true;
    r.principal.owns_control_lease=true;
    assert(recovered.handle(r).http_status==423 &&
           !recovered.review_snapshot(r));
  }
  auto r=get_req();r.principal.authenticated=false;
  assert(recovered.handle(r).http_status==401);
  r=get_req();r.principal.can_read_mission=false;
  assert(recovered.handle(r).http_status==403);
  r=get_req();r.principal.operator_session_id="FORGED";
  assert(recovered.handle(r).http_status==403);
  r=get_req();r.mission_run_id="FORGED_RUN_ID";
  assert(recovered.handle(r).http_status==403);
  r=get_req();r.method="POST";
  assert(recovered.handle(r).http_status==405);
  assert(!g.emits_hardware_commands());
}
static void manual() {
  LocalDb db("manual");
  NativeMissionCommandGateway g;
  NativeBackendMissionUpload b(g);
  persist_upload(db.file,b,MissionMode::manual);checkpoint(db.file);
  NativeOfflineRecoveredDraftPreviewLab recovered(db.file.string());
  assert(recovered.reconstruct_for_review(trusted(),g).state==
         DraftRecoveredPreviewLabState::preview_reconstructed);
  assert(recovered.review_mode(get_req())==MissionMode::manual);
  assert(!g.has_active_mission());
}
static void stale() {
  LocalDb db("stale");
  NativeMissionCommandGateway g;
  NativeBackendMissionUpload b(g);
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  NativeOfflineDraftJournaledUploadLab writer(journal,b);
  auto up=upload_req(kBackendMissionUploadPath,1,10);
  auto first=writer.submit(proposal(up,1),up);
  assert(first.journal_completed && first.backend_reply.http_status==200);
  checkpoint(db.file);
  auto upd=update_req(first.backend_reply,2,11);
  upd.marking_points[1].x=3.0;
  assert(writer.submit(proposal(upd,2),upd).journal_completed);
  NativeOfflineRecoveredDraftPreviewLab recovered(db.file.string());
  assert(recovered.reconstruct_for_review(trusted(),g).state==
         DraftRecoveredPreviewLabState::preflight_rejected);
  assert(!recovered.has_recovered_preview());
  assert(recovered.handle(get_req()).http_status==404);
}
static void pending() {
  LocalDb db("pending");
  NativeMissionCommandGateway g;
  NativeBackendMissionUpload b(g);
  const auto first=persist_upload(db.file,b);
  checkpoint(db.file);
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  auto upd=update_req(first,2,11);
  assert(journal.begin(proposal(upd,2)));
  NativeOfflineRecoveredDraftPreviewLab recovered(db.file.string());
  assert(recovered.reconstruct_for_review(trusted(),g).state==
         DraftRecoveredPreviewLabState::preflight_rejected);
}
static void ready() {
  LocalDb db("ready");
  NativeMissionCommandGateway g;
  NativeBackendMissionUpload b(g);
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  NativeOfflineDraftJournaledUploadLab writer(journal,b);
  auto up=upload_req(kBackendMissionUploadPath,1,10);
  auto first=writer.submit(proposal(up,1),up);
  assert(first.journal_completed && first.backend_reply.http_status==200);
  checkpoint(db.file);
  auto load=load_req(first.backend_reply,2,11);
  assert(writer.submit(proposal(load,2),load).journal_completed);
  // With an active gateway the detached lab fails before accessing SQLite.
  NativeOfflineRecoveredDraftPreviewLab recovered(db.file.string());
  assert(recovered.reconstruct_for_review(trusted(),g).state==
         DraftRecoveredPreviewLabState::gateway_active);
  // Even without gateway state, a READY record cannot reconstruct PREVIEW.
  NativeMissionCommandGateway fresh_gateway;
  assert(recovered.reconstruct_for_review(trusted(),fresh_gateway).state==
         DraftRecoveredPreviewLabState::preflight_rejected);
}
static void missing() {
  LocalDb db("missing");
  NativeMissionCommandGateway g;
  NativeBackendMissionUpload b(g);
  persist_upload(db.file,b);
  NativeOfflineRecoveredDraftPreviewLab recovered(db.file.string());
  assert(recovered.reconstruct_for_review(trusted(),g).state==
         DraftRecoveredPreviewLabState::preflight_rejected);
  auto r=trusted();r.explicit_operator_review=false;
  assert(recovered.reconstruct_for_review(r,g).state==
         DraftRecoveredPreviewLabState::preflight_rejected);
  r=trusted();r.independently_disarmed=false;
  assert(recovered.reconstruct_for_review(r,g).state==
         DraftRecoveredPreviewLabState::preflight_rejected);
  r=trusted();r.proposed_new_epoch=3;
  assert(recovered.reconstruct_for_review(r,g).state==
         DraftRecoveredPreviewLabState::preflight_rejected);
  NativeOfflineRecoveredDraftPreviewLab bogus("/tmp/dyx_h5g_missing_db_do_not_create.sqlite");
  assert(bogus.reconstruct_for_review(trusted(),g).state==
         DraftRecoveredPreviewLabState::storage_error);
}
int main(int argc,char**argv){
  assert(argc==2);
  const std::string mode=argv[1];
  if(mode=="nominal")nominal();
  else if(mode=="restart")restart();
  else if(mode=="requests")requests();
  else if(mode=="manual")manual();
  else if(mode=="stale")stale();
  else if(mode=="pending")pending();
  else if(mode=="ready")ready();
  else if(mode=="missing")missing();
  else return 2;
  std::cout<<"PASS H5G isolated no-actuation PREVIEW facade "<<mode<<'\n';
}
