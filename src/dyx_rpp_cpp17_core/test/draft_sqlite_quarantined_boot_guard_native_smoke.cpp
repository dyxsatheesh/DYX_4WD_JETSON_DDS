// DYX H5K synthetic SQLite laboratory; no hardware, no real HTTP network.
#include "backend_upload_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_quarantined_boot_guard_lab.hpp"
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
  explicit TempDb(const std::string& name) {
    const auto n=std::chrono::steady_clock::now().time_since_epoch().count();
    dir=fs::temp_directory_path()/("dyx_h5k_"+name+"_"+std::to_string(n));
    assert(fs::create_directory(dir));file=dir/"mission.sqlite";
    assert(NativeDraftSqliteStoreLab::provision_new(file.string(),
      {"SESSION_H4E_OPERATOR",3,0,0.0,1}));
    assert(NativeDraftSqliteMutationJournalLab::provision_schema_for_existing_store(
      file.string()));
  }
  ~TempDb(){std::error_code ec;fs::remove_all(dir,ec);}
};
static DraftRecoveryPreflightLabRequest preflight() {
  DraftRecoveryPreflightLabRequest r;
  r.saved_identity={true,true,true,"SESSION_H4E_OPERATOR",3};
  r.proposed_new_epoch=4;r.explicit_operator_review=true;
  r.independently_disarmed=true;r.independently_stationary=true;
  r.independently_no_active_mission=true;return r;
}
static DraftQuarantinedHandoffLabRequest staging() {
  DraftQuarantinedHandoffLabRequest r;
  r.current_identity={true,true,true,"SESSION_H4E_OPERATOR",4};
  r.explicit_operator_review=true;r.independently_disarmed=true;
  r.independently_stationary=true;
  r.independently_no_active_mission=true;return r;
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
static void prepare(TempDb& db,bool stage=true) {
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  NativeOfflineDraftJournaledUploadLab writer(journal,b);
  const auto req=upload_req(kBackendMissionUploadPath,1,10);
  const auto result=writer.submit(proposal(req,1),req);
  assert(result.journal_completed && result.backend_reply.http_status==200);
  assert(NativeOfflineDraftPreviewCheckpointLab(db.file.string())
      .seal_preview(preflight().saved_identity).state==
      DraftPreviewCheckpointLabState::sealed_preview);
  NativeMissionCommandGateway fresh;NativeBackendMissionUpload empty(fresh);
  assert(NativeOfflineDraftEpochQuarantineLab(db.file.string())
      .quarantine(preflight(),fresh,empty).state==
      DraftEpochQuarantineLabState::quarantine_committed);
  if(stage)assert(NativeOfflineQuarantinedPreviewHandoffLab(db.file.string())
      .stage_for_review(staging(),fresh,empty).state==
      DraftQuarantinedHandoffLabState::staged_preview);
}
static void sql(TempDb& db,const char* statement) {
  sqlite3* s=draft_journal_lab_detail::open_existing(db.file.string());
  assert(s && draft_journal_lab_detail::exec(s,statement));
  sqlite3_close_v2(s);
}
static BackendMissionUploadRequest get() {
  auto r=upload_req(kBackendMissionDraftPath,0,0);
  r.method="GET";r.marking_points.clear();return r;
}
static void nominal() {
  TempDb db("nominal");prepare(db);
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  NativeOfflineQuarantinedBootGuardLab guard(db.file.string(),g,b);
  const auto status=guard.begin(review());
  assert(status.can_read_preview && !status.can_load &&
     !status.can_start_or_resume && !status.can_upload_or_update &&
     !status.can_unquarantine && !status.hardware_command_emitted &&
     status.state==DraftQuarantinedBootGuardLabState::read_only_preview);
  const auto v=guard.inspect_current(review());
  assert(v.read_only_snapshot && v.read_only_snapshot->marking_points.size()==2 &&
     v.mission_mode && *v.mission_mode==MissionMode::automatic);
  assert(!b.has_draft() && !g.has_active_mission() && !g.emits_hardware_commands());
}
static void restart() {
  TempDb db("restart");prepare(db);
  for(int i=0;i<3;++i) {
    NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
    NativeOfflineQuarantinedBootGuardLab guard(db.file.string(),g,b);
    assert(guard.begin(review()).can_read_preview);
    assert(guard.handle(get(),review()).http_status==200);
    assert(!b.has_draft());
  }
}
static void uninitialized() {
  TempDb db("uninitialized");prepare(db);
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  NativeOfflineQuarantinedBootGuardLab guard(db.file.string(),g,b);
  assert(guard.handle(get(),review()).http_status==423);
  assert(guard.inspect_current(review()).state==DraftQuarantinedReviewLabState::denied);
  assert(!b.has_draft());
}
static void writes() {
  TempDb db("writes");prepare(db);
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  NativeOfflineQuarantinedBootGuardLab guard(db.file.string(),g,b);
  assert(guard.begin(review()).can_read_preview);
  for(const auto* route : {kBackendMissionUploadPath,kBackendMissionUpdatePath,
                           kBackendMissionLoadPath,"/api/v1/mission/start",
                           "/api/v1/mission/resume"}) {
    auto r=upload_req(route,2,11);
    assert(guard.handle(r,review()).http_status==423);
  }
  assert(!b.has_draft() && !g.has_active_mission());
  // STOP must have a separate independent safety path. This *lab* models
  // only upload/draft review and must never be installed as STOP router.
}
static void readonly() {
  TempDb db("readonly");prepare(db);
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  NativeOfflineQuarantinedBootGuardLab guard(db.file.string(),g,b);
  assert(guard.begin(review()).can_read_preview);
  auto r=get();assert(guard.handle(r,review()).http_status==200);
  r.method="POST";assert(guard.handle(r,review()).http_status==423);
  r=get();r.marking_points={{1,1}};assert(guard.handle(r,review()).http_status==423);
  r=get();r.sequence=7;assert(guard.handle(r,review()).http_status==423);
  r=get();r.expected_signature="fake";
  assert(guard.handle(r,review()).http_status==423);
  assert(!b.has_draft());
}
static void identity() {
  TempDb db("identity");prepare(db);
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  NativeOfflineQuarantinedBootGuardLab guard(db.file.string(),g,b);
  assert(guard.begin(review()).can_read_preview);
  auto r=get();r.principal.operator_session_id="OTHER";
  assert(guard.handle(r,review()).http_status==423);
  r=get();r.mission_run_id="OTHER";
  assert(guard.handle(r,review()).http_status==423);
  r=get();r.principal.authenticated=false;
  assert(guard.handle(r,review()).http_status==423);
  auto claim=review();claim.current_identity.session_epoch=3;
  assert(guard.inspect_current(claim).state==DraftQuarantinedReviewLabState::denied);
  assert(guard.handle(get(),claim).http_status==423);
  // A bad caller does not poison a valid review; durable tamper does.
  assert(guard.handle(get(),review()).http_status==200);
}
static void tamper() {
  TempDb db("tamper");prepare(db);
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  NativeOfflineQuarantinedBootGuardLab guard(db.file.string(),g,b);
  assert(guard.begin(review()).can_read_preview);
  sql(db,"UPDATE dyx_mission_recovery_staging SET last_sequence=99");
  assert(guard.handle(get(),review()).http_status==423);
  assert(guard.state()==DraftQuarantinedBootGuardLabState::locked_permanently);
  assert(!guard.begin(review()).can_read_preview);
  assert(!b.has_draft());
}
static void active() {
  TempDb db("active");prepare(db);
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  NativeOfflineQuarantinedBootGuardLab guard(db.file.string(),g,b);
  assert(guard.begin(review()).can_read_preview);
  // External *direct* backend mutation bypasses this lab, proving why the
  // production ingress MUST enforce sole ownership of backend mutation.
  assert(b.handle(upload_req(kBackendMissionUploadPath,1,11)).http_status==200);
  assert(guard.handle(get(),review()).http_status==423);
  assert(guard.state()==DraftQuarantinedBootGuardLabState::locked_permanently);
  assert(b.has_draft());
}
static void failed_boot() {
  TempDb db("failed_boot");prepare(db,false);
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  NativeOfflineQuarantinedBootGuardLab guard(db.file.string(),g,b);
  assert(!guard.begin(review()).can_read_preview);
  assert(guard.state()==DraftQuarantinedBootGuardLabState::locked_permanently);
  assert(!guard.begin(review()).can_read_preview);
  assert(guard.handle(get(),review()).http_status==423);
}
static void retirement() {
  TempDb db("retirement");prepare(db);
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  NativeOfflineQuarantinedBootGuardLab guard(db.file.string(),g,b);
  assert(guard.begin(review()).can_read_preview);
  assert(!guard.begin(review()).can_read_preview);
  guard.retire();
  assert(guard.handle(get(),review()).http_status==423);
  assert(!guard.begin(review()).can_read_preview);
}
static void journal() {
  TempDb db("journal");prepare(db);
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  NativeOfflineQuarantinedBootGuardLab guard(db.file.string(),g,b);
  assert(guard.begin(review()).can_read_preview);
  NativeDraftSqliteMutationJournalLab writer(db.file.string());
  const auto req=upload_req(kBackendMissionUploadPath,2,11);
  auto p=proposal(req,2);
  assert(!writer.begin(p)); // stale epoch
  p.session_epoch=4;
  assert(!writer.begin(p)); // current epoch is still QUARANTINED
  assert(guard.handle(get(),review()).http_status==200);
  assert(!b.has_draft());
}
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  const std::string m=argv[1];
  if(m=="nominal")nominal();
  else if(m=="restart")restart();
  else if(m=="uninitialized")uninitialized();
  else if(m=="writes")writes();
  else if(m=="readonly")readonly();
  else if(m=="identity")identity();
  else if(m=="tamper")tamper();
  else if(m=="active")active();
  else if(m=="failed_boot")failed_boot();
  else if(m=="retirement")retirement();
  else if(m=="journal")journal();
  else return 2;
  std::cout<<"PASS H5K "<<m<<"\n";
}
