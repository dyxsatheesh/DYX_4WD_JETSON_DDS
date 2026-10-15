// DYX H5K synthetic SQLite laboratory; no hardware, no real HTTP network.
#include "backend_upload_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_recovery_startup_owner_lab.hpp"
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

static ClientCommandRequest cmd(ClientMissionAction action,
                                bool authenticated=true) {
  ClientCommandRequest r;
  r.action=action;
  r.authenticated_operator=authenticated;
  r.authorized_control_lease=true;
  r.mission_run_id="RUN_H4E_0001";
  r.operator_session_id="SESSION_H4E_OPERATOR";
  r.session_sequence=1;
  r.received_at_host_sec=12.0;
  return r;
}
static void nominal() {
  TempDb db("owner_nominal");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(), review());
  auto s=owner.status();
  assert(s.quarantined && s.read_only_preview_possible &&
      !s.can_start_or_resume && !s.can_upload_or_load &&
      !s.physical_stop_confirmed && !s.hardware_command_emitted);
  assert(!owner.has_active_mission() && !owner.has_live_backend_draft());
  assert(owner.handle_draft(get(),review()).http_status==200);
  assert(owner.emits_hardware_commands()==false);
}
static void restart() {
  TempDb db("owner_restart");prepare(db);
  for(int i=0;i<3;++i) {
    NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
    assert(owner.status().quarantined);
    assert(owner.handle_draft(get(),review()).http_status==200);
    assert(owner.handle_command(cmd(ClientMissionAction::start)).status==
           ClientCommandStatus::recovery_quarantined);
    assert(!owner.has_active_mission() && !owner.has_live_backend_draft());
  }
}
static void missing() {
  TempDb db("owner_missing");prepare(db,false);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  assert(owner.status().quarantined);
  assert(!owner.status().read_only_preview_possible);
  assert(owner.handle_draft(get(),review()).http_status==423);
  assert(owner.handle_command(cmd(ClientMissionAction::start)).status==
         ClientCommandStatus::recovery_quarantined);
  // The DB file itself is missing: never create a new authorizing state.
  const auto nonexistent=(db.dir/"missing.sqlite").string();
  NativeOfflineRecoveryStartupOwnerLab absent(nonexistent, review());
  assert(absent.status().quarantined);
  assert(!absent.status().read_only_preview_possible);
  assert(absent.handle_draft(get(),review()).http_status==423);
  assert(!fs::exists(nonexistent));
}
static void writes() {
  TempDb db("owner_writes");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  for(const auto* route : {kBackendMissionUploadPath,kBackendMissionUpdatePath,
                           kBackendMissionLoadPath,"/api/v1/mission/start"}) {
    auto r=upload_req(route,5,13);
    assert(owner.handle_draft(r,review()).http_status==423);
  }
  for(const auto a : {ClientMissionAction::start,ClientMissionAction::pause,
                     ClientMissionAction::resume,ClientMissionAction::next,
                     ClientMissionAction::skip})
    assert(owner.handle_command(cmd(a)).status==
           ClientCommandStatus::recovery_quarantined);
  assert(!owner.has_live_backend_draft() && !owner.has_active_mission());
}
static void stop() {
  TempDb db("owner_stop");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  auto unauth=cmd(ClientMissionAction::stop,false);
  assert(owner.handle_command(unauth).status==ClientCommandStatus::unauthorized);
  auto stop=owner.handle_command(cmd(ClientMissionAction::stop));
  assert(stop.status==ClientCommandStatus::accepted);
  assert(!stop.hardware_command_emitted && owner.status().quarantined);
  assert(owner.handle_command(cmd(ClientMissionAction::start)).status==
         ClientCommandStatus::recovery_quarantined);
}
static void tamper() {
  TempDb db("owner_tamper");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  assert(owner.handle_draft(get(),review()).http_status==200);
  sql(db,"UPDATE dyx_mission_recovery_staging SET last_sequence=999");
  assert(owner.handle_draft(get(),review()).http_status==423);
  assert(!owner.status().read_only_preview_possible);
  assert(owner.handle_draft(get(),review()).http_status==423);
  NativeOfflineRecoveryStartupOwnerLab after(db.file.string(),review());
  assert(after.status().quarantined && !after.status().read_only_preview_possible);
}
static void identity() {
  TempDb db("owner_identity");prepare(db);
  auto bad=review();bad.explicit_operator_review=false;
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),bad);
  assert(owner.status().quarantined);
  assert(!owner.status().read_only_preview_possible);
  assert(owner.handle_draft(get(),review()).http_status==423);
  assert(owner.handle_command(cmd(ClientMissionAction::resume)).status==
         ClientCommandStatus::recovery_quarantined);
}
static void journal() {
  TempDb db("owner_journal");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  NativeDraftSqliteMutationJournalLab j(db.file.string());
  const auto req=upload_req(kBackendMissionUploadPath,2,15);
  auto p=proposal(req,2);
  assert(!j.begin(p));
  p.session_epoch=4;
  assert(!j.begin(p));
  assert(owner.handle_draft(get(),review()).http_status==200);
}
static void retirement() {
  TempDb db("owner_retirement");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  assert(owner.handle_draft(get(),review()).http_status==200);
  owner.retire();
  assert(owner.handle_draft(get(),review()).http_status==423);
  assert(owner.status().quarantined);
  assert(owner.handle_command(cmd(ClientMissionAction::start)).status==
         ClientCommandStatus::recovery_quarantined);
}
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  const std::string m=argv[1];
  if(m=="nominal")nominal();
  else if(m=="restart")restart();
  else if(m=="missing")missing();
  else if(m=="writes")writes();
  else if(m=="stop")stop();
  else if(m=="tamper")tamper();
  else if(m=="identity")identity();
  else if(m=="journal")journal();
  else if(m=="retirement")retirement();
  else return 2;
  std::cout<<"PASS H5N "<<m<<"\n";
}
