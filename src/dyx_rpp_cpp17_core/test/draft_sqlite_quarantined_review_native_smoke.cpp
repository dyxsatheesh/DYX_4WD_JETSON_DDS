// DYX Autonomous H5J: detached read-only review; no PX4/DDS/actuator.
#include "backend_upload_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_quarantined_review_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_sqlite_store_lab.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
namespace fs=std::filesystem;
using namespace dyx::rpp;
struct TempDb {
  fs::path dir,file;
  explicit TempDb(const std::string& mode) {
    const auto n=std::chrono::steady_clock::now().time_since_epoch().count();
    dir=fs::temp_directory_path()/("dyx_h5j_"+mode+"_"+std::to_string(n));
    assert(fs::create_directory(dir));file=dir/"mission.sqlite";
    assert(NativeDraftSqliteStoreLab::provision_new(file.string(),
       {"SESSION_H4E_OPERATOR",3,0,0.0,1}));
    assert(NativeDraftSqliteMutationJournalLab::provision_schema_for_existing_store(file.string()));
  }
  ~TempDb(){std::error_code e;fs::remove_all(dir,e);}
};
static DraftRecoveryPreflightLabRequest preflight() {
  DraftRecoveryPreflightLabRequest r;
  r.saved_identity={true,true,true,"SESSION_H4E_OPERATOR",3};
  r.proposed_new_epoch=4;r.explicit_operator_review=true;
  r.independently_disarmed=true;r.independently_stationary=true;
  r.independently_no_active_mission=true;return r;
}
static DraftQuarantinedHandoffLabRequest handoff() {
  DraftQuarantinedHandoffLabRequest r;
  r.current_identity={true,true,true,"SESSION_H4E_OPERATOR",4};
  r.explicit_operator_review=true;r.independently_disarmed=true;
  r.independently_stationary=true;r.independently_no_active_mission=true;
  return r;
}
static DraftQuarantinedReviewLabRequest review() {
  DraftQuarantinedReviewLabRequest r;
  r.current_identity={true,true,true,"SESSION_H4E_OPERATOR",4};
  r.explicit_operator_review=true;return r;
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
static void prepare(TempDb& db,bool do_stage=true) {
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  NativeMissionCommandGateway gateway;NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(journal,backend);
  const auto r=upload_req(kBackendMissionUploadPath,1,10);
  const auto saved=writer.submit(proposal(r,1),r);
  assert(saved.journal_completed && saved.backend_reply.http_status==200);
  assert(NativeOfflineDraftPreviewCheckpointLab(db.file.string())
        .seal_preview(preflight().saved_identity).state==
        DraftPreviewCheckpointLabState::sealed_preview);
  // A fresh empty backend is required for quarantine and handoff.
  NativeMissionCommandGateway fresh;NativeBackendMissionUpload empty(fresh);
  assert(NativeOfflineDraftEpochQuarantineLab(db.file.string())
        .quarantine(preflight(),fresh,empty).state==
        DraftEpochQuarantineLabState::quarantine_committed);
  if(do_stage)assert(NativeOfflineQuarantinedPreviewHandoffLab(db.file.string())
        .stage_for_review(handoff(),fresh,empty).state==
        DraftQuarantinedHandoffLabState::staged_preview);
}
static void sql(TempDb& db,const char* query) {
  sqlite3* d=draft_journal_lab_detail::open_existing(db.file.string());
  assert(draft_journal_lab_detail::exec(d,query));sqlite3_close_v2(d);
}
static DraftQuarantinedReviewLabResult inspect(TempDb& db,
       DraftQuarantinedReviewLabRequest request=review()) {
  NativeMissionCommandGateway gateway;NativeBackendMissionUpload backend(gateway);
  return NativeOfflineQuarantinedPreviewReviewLab(db.file.string())
        .inspect(request,gateway,backend);
}
static void nominal(){
  TempDb db("nominal");prepare(db);
  const auto r=inspect(db);
  assert(r.state==DraftQuarantinedReviewLabState::reviewed_preview_only &&
         r.old_epoch==3 && r.quarantined_epoch==4 && r.nonce==1 &&
         r.revision==1 && r.last_sequence==1 &&
         r.read_only_snapshot && r.read_only_snapshot->marking_points.size()==2 &&
         r.mission_mode && *r.mission_mode==MissionMode::automatic &&
         !r.can_post && !r.can_unquarantine && !r.can_load &&
         !r.can_start_or_resume && !r.hardware_command_emitted);
}
static void restart(){
  TempDb db("restart");prepare(db);
  for(int i=0;i<2;++i)assert(inspect(db).state==
         DraftQuarantinedReviewLabState::reviewed_preview_only);
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  const auto r=upload_req(kBackendMissionUploadPath,2,11);
  auto p=proposal(r,2);p.session_epoch=4;
  assert(!journal.begin(p));
}
static void authorization(){
  TempDb db("auth");prepare(db);
  auto r=review();r.current_identity.authenticated=false;
  assert(inspect(db,r).state==DraftQuarantinedReviewLabState::denied);
  r=review();r.current_identity.can_read_mission=false;
  assert(inspect(db,r).state==DraftQuarantinedReviewLabState::denied);
  r=review();r.explicit_operator_review=false;
  assert(inspect(db,r).state==DraftQuarantinedReviewLabState::denied);
  r=review();r.current_identity.session_epoch=3;
  assert(inspect(db,r).state==DraftQuarantinedReviewLabState::quarantine_mismatch);
  r=review();r.current_identity.operator_session_id="ATTACKER";
  assert(inspect(db,r).state==DraftQuarantinedReviewLabState::quarantine_mismatch);
}
static void missing(){
  TempDb db("missing");prepare(db,false);
  assert(inspect(db).state==DraftQuarantinedReviewLabState::staging_missing);
}
static void stage_tamper(){
  TempDb db("stage_tamper");prepare(db);
  sql(db,"UPDATE dyx_mission_recovery_staging SET last_sequence=42");
  assert(inspect(db).state==DraftQuarantinedReviewLabState::staging_mismatch);
}
static void barrier_tamper(){
  TempDb db("barrier_tamper");prepare(db);
  sql(db,"UPDATE dyx_mission_recovery_epoch_barrier SET new_epoch=5");
  assert(inspect(db).state==DraftQuarantinedReviewLabState::quarantine_mismatch);
}
static void history_tamper(){
  TempDb db("history_tamper");prepare(db);
  sql(db,"UPDATE dyx_mission_mutations SET request_json='{}' WHERE nonce=1");
  assert(inspect(db).state==DraftQuarantinedReviewLabState::history_rejected);
}
static void active(){
  TempDb db("active");prepare(db);
  NativeMissionCommandGateway gateway;NativeBackendMissionUpload backend(gateway);
  const auto upload=upload_req(kBackendMissionUploadPath,1,10);
  assert(backend.handle(upload).http_status==200 && backend.has_draft());
  assert(NativeOfflineQuarantinedPreviewReviewLab(db.file.string())
        .inspect(review(),gateway,backend).state==
        DraftQuarantinedReviewLabState::backend_active);
}
static void locked(){
  TempDb db("locked");prepare(db);
  NativeMissionCommandGateway gateway;NativeBackendMissionUpload backend(gateway);
  NativeOfflineQuarantinedPreviewReviewLab view(db.file.string());
  for(const auto& route : {kBackendMissionUploadPath,kBackendMissionUpdatePath,
                           kBackendMissionLoadPath,
                           "/api/v1/mission/start", "/api/v1/mission/resume"}) {
    auto request=upload_req(route,2,11);
    assert(view.handle(request,review(),gateway,backend).http_status==423);
  }
  auto r=upload_req(kBackendMissionDraftPath,0,0);r.method="GET";r.marking_points.clear();
  assert(view.handle(r,review(),gateway,backend).http_status==200);
  r.principal.operator_session_id="FORGED";
  assert(view.handle(r,review(),gateway,backend).http_status==403);
  r=upload_req(kBackendMissionDraftPath,0,0);r.method="GET";r.marking_points.clear();r.principal.authenticated=false;
  assert(view.handle(r,review(),gateway,backend).http_status==401);
  r=upload_req(kBackendMissionDraftPath,0,0);r.method="POST";r.marking_points.clear();
  assert(view.handle(r,review(),gateway,backend).http_status==405);
  assert(!gateway.emits_hardware_commands() && !backend.has_draft());
}
int main(int argc,char** argv){
  if(argc!=2)return 2;
  const std::string mode=argv[1];
  if(mode=="nominal")nominal();
  else if(mode=="restart")restart();
  else if(mode=="authorization")authorization();
  else if(mode=="missing")missing();
  else if(mode=="stage_tamper")stage_tamper();
  else if(mode=="barrier_tamper")barrier_tamper();
  else if(mode=="history_tamper")history_tamper();
  else if(mode=="active")active();
  else if(mode=="locked")locked();
  else return 2;
  std::cout<<"PASS H5J "<<mode<<"\n";
}
