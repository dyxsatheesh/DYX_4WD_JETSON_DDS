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

static BackendTrustedHttpRequest http_request(const std::string& method,
                                              const std::string& path) {
  BackendTrustedHttpRequest q;
  q.method=method;q.path=path;q.host_received_sec=12.0;
  q.principal.authenticated=true;
  q.principal.can_read_mission=true;
  q.principal.owns_control_lease=true;
  q.principal.operator_session_id="SESSION_H4E_OPERATOR";
  if(method=="POST")
    q.body="{\"mission_run_id\":\"RUN_H4E_0001\",\"sequence\":1}";
  return q;
}
static BackendTrustedUploadHttpRequest draft_request() {
  BackendTrustedUploadHttpRequest input;
  input.http=http_request("GET",std::string(kBackendMissionDraftPath)+
      "?mission_run_id=RUN_H4E_0001");
  input.trusted_px4_disarmed=false;
  return input;
}
static void status() {
  TempDb db("http_status");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  const auto q=http_request("GET",kBackendMissionStatusPath);
  const auto r=owner.handle_http(q);
  assert(r.status==200);
  assert(r.body.find("\"recovery_quarantined\":true")!=std::string::npos);
  assert(r.http11_bytes().find("HTTP/1.1 200 OK\r\n")==0);
  assert(!r.physical_command_emitted && !r.network_io_performed);
  assert(owner.status().quarantined);
}
static void commands() {
  TempDb db("http_commands");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  for(const auto* route:{kBackendMissionStartPath,kBackendMissionPausePath,
      kBackendMissionResumePath,kBackendMissionNextPath,kBackendMissionSkipPath}) {
    const auto r=owner.handle_http(http_request("POST",route));
    assert(r.status==423 &&
           r.body.find("MISSION_RECOVERY_QUARANTINED")!=std::string::npos);
  }
  assert(!owner.has_active_mission() && !owner.has_live_backend_draft());
}
static void stop() {
  TempDb db("http_stop");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  auto req=http_request("POST",kBackendMissionStopPath);
  req.principal.owns_control_lease=false;
  assert(owner.handle_http(req).status==200);
  assert(owner.status().quarantined);
  assert(owner.handle_http(http_request("POST",kBackendMissionStartPath)).status==423);
  assert(!owner.emits_hardware_commands());
}
static void preview() {
  TempDb db("http_preview");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  const auto r=owner.handle_recovered_draft_http(draft_request(),review());
  assert(r.status==200 &&
         r.body.find("QUARANTINED_PREVIEW_READ_ONLY")!=std::string::npos &&
         r.body.find("\"preview_only\":true")!=std::string::npos &&
         r.body.find("\"hardware_command_emitted\":false")!=std::string::npos);
  assert(r.body.find("marking_points")==std::string::npos);
  assert(!owner.has_active_mission() && !owner.has_live_backend_draft());
}
static void auth() {
  TempDb db("http_auth");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  auto q=http_request("GET",kBackendMissionStatusPath);
  q.principal.authenticated=false;
  assert(owner.handle_http(q).status==401);
  auto d=draft_request();d.http.principal.authenticated=false;
  assert(owner.handle_recovered_draft_http(d,review()).status==401);
  d=draft_request();d.http.principal.can_read_mission=false;
  assert(owner.handle_recovered_draft_http(d,review()).status==403);
  d=draft_request();d.http.principal.operator_session_id="SESSION_FAKE";
  assert(owner.handle_recovered_draft_http(d,review()).status==403);
}
static void malformed() {
  TempDb db("http_malformed");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  auto d=draft_request();d.http.path+="&override=1";
  assert(owner.handle_recovered_draft_http(d,review()).status==400);
  d=draft_request();d.http.body="{}";
  assert(owner.handle_recovered_draft_http(d,review()).status==400);
  d=draft_request();d.http.method="POST";
  assert(owner.handle_recovered_draft_http(d,review()).status==405);
  d=draft_request();d.http.host_received_sec=-1;
  assert(owner.handle_recovered_draft_http(d,review()).status==400);
  d=draft_request();d.http.path=kBackendMissionUploadPath;
  assert(owner.handle_recovered_draft_http(d,review()).status==404);
  assert(owner.status().quarantined);
}
static void tamper() {
  TempDb db("http_tamper");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  assert(owner.handle_recovered_draft_http(draft_request(),review()).status==200);
  sql(db,"UPDATE dyx_mission_recovery_staging SET last_sequence=999");
  assert(owner.handle_recovered_draft_http(draft_request(),review()).status==423);
  assert(owner.handle_recovered_draft_http(draft_request(),review()).status==423);
  assert(owner.handle_http(http_request("POST",kBackendMissionStartPath)).status==423);
}
static void locked() {
  TempDb db("http_locked");prepare(db,false);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  assert(owner.handle_recovered_draft_http(draft_request(),review()).status==423);
  assert(owner.handle_http(http_request("POST",kBackendMissionStartPath)).status==423);
  assert(owner.status().quarantined);
}
static void retired() {
  TempDb db("http_retired");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  owner.retire();
  assert(owner.handle_recovered_draft_http(draft_request(),review()).status==423);
  assert(owner.handle_http(http_request("POST",kBackendMissionStartPath)).status==423);
  assert(!owner.has_active_mission());
}
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  const std::string m=argv[1];
  if(m=="status")status();
  else if(m=="commands")commands();
  else if(m=="stop")stop();
  else if(m=="preview")preview();
  else if(m=="auth")auth();
  else if(m=="malformed")malformed();
  else if(m=="tamper")tamper();
  else if(m=="locked")locked();
  else if(m=="retired")retired();
  else return 2;
  std::cout<<"PASS H5O "<<m<<"\n";
}
