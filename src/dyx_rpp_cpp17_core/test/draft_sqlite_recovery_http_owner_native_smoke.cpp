// DYX H5K synthetic SQLite laboratory; no hardware, no real HTTP network.
#include "backend_upload_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_recovery_startup_owner_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_recovery_service_lifecycle_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_recovery_trusted_ingress_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_recovery_host_supervisor_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_sqlite_store_lab.hpp"
#include <cassert>
#include <atomic>
#include <thread>
#include <vector>
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
// H5P: exercise the canonical owner-bound recovery ingress rather than
// calling mission and draft handlers through separate simulated adapters.
static BackendTrustedUploadHttpRequest ingress(const std::string& method,
                                                const std::string& path) {
  BackendTrustedUploadHttpRequest v;
  v.http=http_request(method,path);
  return v;
}
static void ingress_status() {
  TempDb db("ingress_status");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  auto r=owner.handle_recovery_ingress(ingress("GET",kBackendMissionStatusPath),review());
  assert(r.status==200 && r.body.find("\"recovery_quarantined\":true")!=std::string::npos);
  assert(!r.physical_command_emitted && !r.network_io_performed);
  assert(r.http11_bytes().find("HTTP/1.1 200 OK\r\n")==0);
  assert(owner.status().quarantined);
}
static void ingress_commands() {
  TempDb db("ingress_commands");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  for(const char* path:{kBackendMissionStartPath,kBackendMissionPausePath,
                       kBackendMissionResumePath,kBackendMissionNextPath,
                       kBackendMissionSkipPath}) {
    const auto r=owner.handle_recovery_ingress(ingress("POST",path),review());
    assert(r.status==423 && r.body.find("MISSION_RECOVERY_QUARANTINED")!=std::string::npos);
    assert(!r.physical_command_emitted);
  }
  assert(!owner.has_active_mission() && !owner.has_live_backend_draft());
}
static void ingress_mutations() {
  TempDb db("ingress_mutations");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  for(const char* path:{kBackendMissionUploadPath,kBackendMissionUpdatePath,
                        kBackendMissionLoadPath,kBackendMissionDraftPath}) {
    auto q=ingress("POST",path);
    q.trusted_px4_disarmed=true; // still no permission to upload/load
    const auto r=owner.handle_recovery_ingress(q,review());
    assert(r.status==423 && r.body.find("RECOVERY_QUARANTINE_COMMANDS_LOCKED")!=std::string::npos);
    assert(r.http11_bytes().find("HTTP/1.1 423 Locked\r\n")==0);
    assert(!r.physical_command_emitted && !r.network_io_performed);
    q.http.method="GET";q.http.body.clear();
    assert(owner.handle_recovery_ingress(q,review()).status==423);
  }
  assert(!owner.has_active_mission() && !owner.has_live_backend_draft());
}
static void ingress_preview() {
  TempDb db("ingress_preview");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  const auto result=owner.handle_recovery_ingress(draft_request(),review());
  assert(result.status==200);
  assert(result.body.find("\"preview_only\":true")!=std::string::npos);
  assert(result.body.find("marking_points")==std::string::npos);
  assert(!owner.has_active_mission() && !owner.has_live_backend_draft());
}
static void ingress_auth() {
  TempDb db("ingress_auth");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  auto q=ingress("POST",kBackendMissionLoadPath);
  q.http.principal.authenticated=false;
  assert(owner.handle_recovery_ingress(q,review()).status==401);
  q=ingress("POST",kBackendMissionStartPath);
  q.http.principal.authenticated=false;
  assert(owner.handle_recovery_ingress(q,review()).status==401);
  auto d=draft_request();d.http.principal.can_read_mission=false;
  assert(owner.handle_recovery_ingress(d,review()).status==403);
  d=draft_request();d.http.principal.operator_session_id="FORGED";
  assert(owner.handle_recovery_ingress(d,review()).status==403);
}
static void ingress_stop() {
  TempDb db("ingress_stop");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  auto q=ingress("POST",kBackendMissionStopPath);
  q.http.principal.owns_control_lease=false;
  q.http.host_received_sec=-1.0;
  const auto r=owner.handle_recovery_ingress(q,review());
  assert(r.status==200 && r.body.find("\"safe_stop_intent\":true")!=std::string::npos);
  assert(owner.status().quarantined);
  assert(owner.handle_recovery_ingress(ingress("POST",kBackendMissionStartPath),review()).status==423);
  assert(!owner.emits_hardware_commands());
}
static void ingress_invalid() {
  TempDb db("ingress_invalid");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  auto q=ingress("POST","/api/v1/mission/nonexistent");
  assert(owner.handle_recovery_ingress(q,review()).status==404);
  q=ingress("POST",kBackendMissionUploadPath);q.http.method="DELETE";
  assert(owner.handle_recovery_ingress(q,review()).status==405);
  q=draft_request();q.http.path+="&duplicate=true";
  assert(owner.handle_recovery_ingress(q,review()).status==400);
  q=draft_request();q.http.method="POST";
  assert(owner.handle_recovery_ingress(q,review()).status==405);
  q=draft_request();q.http.body="{}";
  assert(owner.handle_recovery_ingress(q,review()).status==400);
  assert(owner.status().quarantined);
}
static void ingress_tamper() {
  TempDb db("ingress_tamper");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  assert(owner.handle_recovery_ingress(draft_request(),review()).status==200);
  sql(db,"UPDATE dyx_mission_recovery_staging SET last_sequence=999");
  assert(owner.handle_recovery_ingress(draft_request(),review()).status==423);
  assert(owner.handle_recovery_ingress(draft_request(),review()).status==423);
  assert(owner.handle_recovery_ingress(ingress("POST",kBackendMissionStartPath),review()).status==423);
}
static void ingress_retired() {
  TempDb db("ingress_retired");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  owner.retire();
  assert(owner.handle_recovery_ingress(draft_request(),review()).status==423);
  assert(owner.handle_recovery_ingress(ingress("POST",kBackendMissionStartPath),review()).status==423);
  assert(owner.handle_recovery_ingress(ingress("POST",kBackendMissionLoadPath),review()).status==423);
  assert(!owner.has_active_mission());
}
// H5Q: lifecycle and same-owner concurrency regression (no actual listener).
static void lifecycle_restart() {
  TempDb db("h5q_restart");prepare(db);
  for (int instance=0;instance<3;++instance) {
    NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
    assert(owner.status().quarantined);
    for (int iteration=0;iteration<4;++iteration) {
      const auto status=owner.handle_recovery_ingress(
          ingress("GET",kBackendMissionStatusPath),review());
      assert(status.status==200 &&
          status.body.find("\"recovery_quarantined\":true")!=std::string::npos);
      assert(owner.handle_recovery_ingress(
          ingress("POST",kBackendMissionStartPath),review()).status==423);
    }
    assert(!owner.has_active_mission() && !owner.has_live_backend_draft());
  }
}
static void lifecycle_review() {
  TempDb db("h5q_review");prepare(db);
  for (int instance=0;instance<3;++instance) {
    NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
    for (int iteration=0;iteration<3;++iteration) {
      const auto r=owner.handle_recovery_ingress(draft_request(),review());
      assert(r.status==200 && r.body.find("\"preview_only\":true")!=std::string::npos);
      assert(r.body.find("marking_points")==std::string::npos);
      assert(!r.physical_command_emitted && !r.network_io_performed);
    }
    assert(owner.status().quarantined && !owner.has_active_mission());
  }
}
static void lifecycle_identity() {
  TempDb db("h5q_identity");prepare(db);
  auto forged=review();forged.current_identity.operator_session_id="FORGED";
  NativeOfflineRecoveryStartupOwnerLab denied(db.file.string(),forged);
  assert(denied.status().quarantined && !denied.status().read_only_preview_possible);
  assert(denied.handle_recovery_ingress(draft_request(),review()).status==423);
  assert(denied.handle_recovery_ingress(
      ingress("POST",kBackendMissionResumePath),review()).status==423);
  denied.retire();
  NativeOfflineRecoveryStartupOwnerLab fresh(db.file.string(),review());
  assert(fresh.status().quarantined);
  assert(fresh.handle_recovery_ingress(
      ingress("POST",kBackendMissionStartPath),review()).status==423);
}
static void lifecycle_tamper() {
  TempDb db("h5q_tamper");prepare(db);
  { NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
    assert(owner.handle_recovery_ingress(draft_request(),review()).status==200);
  }
  sql(db,"UPDATE dyx_mission_recovery_staging SET last_sequence=999");
  for(int i=0;i<2;++i) {
    NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
    assert(owner.status().quarantined);
    assert(owner.handle_recovery_ingress(draft_request(),review()).status==423);
    assert(owner.handle_recovery_ingress(
        ingress("POST",kBackendMissionStartPath),review()).status==423);
  }
}
static void lifecycle_missing() {
  TempDb db("h5q_missing");prepare(db);
  fs::remove(db.file);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  assert(owner.status().quarantined);
  assert(owner.handle_recovery_ingress(draft_request(),review()).status==423);
  assert(owner.handle_recovery_ingress(
      ingress("POST",kBackendMissionStartPath),review()).status==423);
  assert(!owner.has_active_mission() && !owner.has_live_backend_draft());
}
static void lifecycle_replays() {
  TempDb db("h5q_replays");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  for(int i=0;i<32;++i) {
    for(const char* route:{kBackendMissionStartPath,kBackendMissionResumePath,
       kBackendMissionNextPath,kBackendMissionSkipPath,kBackendMissionPausePath}) {
      const auto r=owner.handle_recovery_ingress(ingress("POST",route),review());
      assert(r.status==423 && !r.physical_command_emitted);
    }
    for(const char* route:{kBackendMissionUploadPath,kBackendMissionLoadPath}) {
      auto req=ingress("POST",route);req.trusted_px4_disarmed=true;
      assert(owner.handle_recovery_ingress(req,review()).status==423);
    }
  }
  assert(owner.status().quarantined && !owner.has_active_mission());
}
static void lifecycle_stop() {
  TempDb db("h5q_stop");prepare(db);
  for(int instance=0;instance<2;++instance) {
    NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
    for(int i=0;i<20;++i) {
      auto req=ingress("POST",kBackendMissionStopPath);
      req.http.principal.owns_control_lease=false;
      assert(owner.handle_recovery_ingress(req,review()).status==200);
      assert(owner.handle_recovery_ingress(
          ingress("POST",kBackendMissionStartPath),review()).status==423);
    }
    assert(owner.status().quarantined && !owner.emits_hardware_commands());
  }
}
static void lifecycle_parallel() {
  TempDb db("h5q_parallel");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  std::atomic<int> failures{0};
  std::vector<std::thread> threads;
  for(int worker=0;worker<6;++worker) threads.emplace_back([&,worker] {
    for(int i=0;i<18;++i) {
      const int mode=(worker+i)%5;
      int actual=0;
      if(mode==0) actual=owner.handle_recovery_ingress(
          ingress("GET",kBackendMissionStatusPath),review()).status;
      if(mode==1) actual=owner.handle_recovery_ingress(
          ingress("POST",kBackendMissionStartPath),review()).status;
      if(mode==2) actual=owner.handle_recovery_ingress(
          ingress("POST",kBackendMissionLoadPath),review()).status;
      if(mode==3) actual=owner.handle_recovery_ingress(draft_request(),review()).status;
      if(mode==4) actual=owner.handle_recovery_ingress(
          ingress("POST",kBackendMissionStopPath),review()).status;
      const int expected=(mode==1||mode==2)?423:200;
      if(actual!=expected)++failures;
    }
  });
  for(auto& thread:threads)thread.join();
  assert(failures.load()==0);
  assert(owner.status().quarantined && !owner.has_active_mission());
}
static void lifecycle_retire_race() {
  TempDb db("h5q_retire_race");prepare(db);
  NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
  std::atomic<int> failures{0};
  std::vector<std::thread> threads;
  for(int worker=0;worker<4;++worker)threads.emplace_back([&] {
    for(int i=0;i<24;++i) {
      const auto preview=owner.handle_recovery_ingress(draft_request(),review());
      if(preview.status!=200 && preview.status!=423)++failures;
      const auto command=owner.handle_recovery_ingress(
          ingress("POST",kBackendMissionResumePath),review());
      if(command.status!=423)++failures;
    }
  });
  threads.emplace_back([&] {
    owner.retire();owner.retire();
  });
  for(auto& thread:threads)thread.join();
  assert(failures.load()==0);
  assert(owner.status().quarantined && !owner.status().read_only_preview_possible);
  assert(owner.handle_recovery_ingress(draft_request(),review()).status==423);
  assert(owner.handle_recovery_ingress(
      ingress("POST",kBackendMissionStopPath),review()).status==200);
  assert(!owner.has_active_mission());
}
static void lifecycle_retire_restart() {
  TempDb db("h5q_retire_restart");prepare(db);
  { NativeOfflineRecoveryStartupOwnerLab owner(db.file.string(),review());
    assert(owner.handle_recovery_ingress(draft_request(),review()).status==200);
    owner.retire();
    assert(owner.handle_recovery_ingress(draft_request(),review()).status==423);
  }
  NativeOfflineRecoveryStartupOwnerLab fresh(db.file.string(),review());
  assert(fresh.status().quarantined);
  assert(fresh.handle_recovery_ingress(
      ingress("POST",kBackendMissionStartPath),review()).status==423);
  assert(!fresh.has_active_mission() && !fresh.has_live_backend_draft());
}
// H5R: offline process-service admission modeled above one quarantined owner.
// This is not a real socket listener or a privileged authentication service.
static void service_boot_closed() {
  TempDb db("h5r_boot_closed");prepare(db);
  NativeOfflineRecoveryServiceLifecycleLab svc(db.file.string(),review());
  assert(svc.snapshot().state==RecoveryServiceLifecycleLabState::admission_closed);
  for (const auto* route:{kBackendMissionStatusPath,kBackendMissionStartPath,
                           kBackendMissionStopPath,kBackendMissionUploadPath}) {
    const auto r=svc.dispatch_recovery_request(ingress("POST",route),review());
    assert(r.status==423 && r.body.find("RECOVERY_SERVICE_NOT_ACCEPTING")!=std::string::npos);
    assert(!r.physical_command_emitted && !r.network_io_performed);
  }
  assert(svc.dispatch_recovery_request(draft_request(),review()).status==423);
}
static void service_open_status() {
  TempDb db("h5r_open");prepare(db);
  NativeOfflineRecoveryServiceLifecycleLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  assert(!svc.admit_recovery_only());
  const auto st=svc.snapshot();
  assert(st.accepts_recovery_only && !st.can_issue_motion && !st.can_upload_or_load);
  assert(!st.hardware_command_emitted && !st.physical_stop_confirmed);
  auto r=svc.dispatch_recovery_request(ingress("GET",kBackendMissionStatusPath),review());
  assert(r.status==200 && r.body.find("\"recovery_quarantined\":true")!=std::string::npos);
  assert(!r.physical_command_emitted && !r.network_io_performed);
}
static void service_locked_actions() {
  TempDb db("h5r_locked");prepare(db);
  NativeOfflineRecoveryServiceLifecycleLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  for(const auto* p:{kBackendMissionStartPath,kBackendMissionPausePath,
       kBackendMissionResumePath,kBackendMissionNextPath,kBackendMissionSkipPath,
       kBackendMissionUploadPath,kBackendMissionUpdatePath,kBackendMissionLoadPath}) {
    auto r=svc.dispatch_recovery_request(ingress("POST",p),review());
    assert(r.status==423 && !r.physical_command_emitted && !r.network_io_performed);
  }
  auto q=ingress("POST",kBackendMissionStopPath);
  q.http.principal.owns_control_lease=false;
  const auto r=svc.dispatch_recovery_request(q,review());
  assert(r.status==200 && r.body.find("\"safe_stop_intent\":true")!=std::string::npos);
}
static void service_preview() {
  TempDb db("h5r_preview");prepare(db);
  NativeOfflineRecoveryServiceLifecycleLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  auto r=svc.dispatch_recovery_request(draft_request(),review());
  assert(r.status==200 && r.body.find("\"preview_only\":true")!=std::string::npos);
  assert(r.body.find("marking_points")==std::string::npos);
  assert(!r.physical_command_emitted && !r.network_io_performed);
}
static void service_missing_db() {
  TempDb db("h5r_missing");prepare(db);
  const auto missing=db.dir/"never-created.sqlite";
  NativeOfflineRecoveryServiceLifecycleLab svc(missing.string(),review());
  assert(svc.admit_recovery_only()); // Status/STOP only; preview still denied.
  assert(svc.dispatch_recovery_request(draft_request(),review()).status==423);
  assert(svc.dispatch_recovery_request(ingress("POST",kBackendMissionStartPath),review()).status==423);
  assert(svc.dispatch_recovery_request(ingress("GET",kBackendMissionStatusPath),review()).status==200);
}
static void service_auth() {
  TempDb db("h5r_auth");prepare(db);
  NativeOfflineRecoveryServiceLifecycleLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  auto q=ingress("POST",kBackendMissionStopPath);q.http.principal.authenticated=false;
  assert(svc.dispatch_recovery_request(q,review()).status==401);
  q=draft_request();q.http.principal.can_read_mission=false;
  assert(svc.dispatch_recovery_request(q,review()).status==403);
  q=draft_request();q.http.principal.operator_session_id="FORGED";
  assert(svc.dispatch_recovery_request(q,review()).status==403);
  assert(svc.dispatch_recovery_request(ingress("GET","/api/v1/unknown"),review()).status==404);
}
static void service_tamper() {
  TempDb db("h5r_tamper");prepare(db);
  NativeOfflineRecoveryServiceLifecycleLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  assert(svc.dispatch_recovery_request(draft_request(),review()).status==200);
  sql(db,"UPDATE dyx_mission_recovery_staging SET last_sequence=999");
  for(int i=0;i<3;++i)
    assert(svc.dispatch_recovery_request(draft_request(),review()).status==423);
  assert(svc.dispatch_recovery_request(ingress("POST",kBackendMissionStartPath),review()).status==423);
}
static void service_shutdown() {
  TempDb db("h5r_shutdown");prepare(db);
  NativeOfflineRecoveryServiceLifecycleLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  svc.shutdown();svc.shutdown();
  assert(svc.snapshot().state==RecoveryServiceLifecycleLabState::permanently_closed);
  assert(!svc.admit_recovery_only());
  for (const auto* route:{kBackendMissionStatusPath,kBackendMissionStartPath,
                          kBackendMissionStopPath,kBackendMissionLoadPath})
    assert(svc.dispatch_recovery_request(ingress("POST",route),review()).status==423);
  assert(svc.dispatch_recovery_request(draft_request(),review()).status==423);
}
static void service_restart() {
  TempDb db("h5r_restart");prepare(db);
  for(int i=0;i<3;++i) {
    NativeOfflineRecoveryServiceLifecycleLab svc(db.file.string(),review());
    assert(svc.dispatch_recovery_request(draft_request(),review()).status==423);
    assert(svc.admit_recovery_only());
    assert(svc.dispatch_recovery_request(draft_request(),review()).status==200);
    assert(svc.dispatch_recovery_request(ingress("POST",kBackendMissionResumePath),review()).status==423);
    svc.shutdown();
    assert(svc.dispatch_recovery_request(draft_request(),review()).status==423);
  }
}
static void service_parallel() {
  TempDb db("h5r_parallel");prepare(db);
  NativeOfflineRecoveryServiceLifecycleLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  std::atomic<int> ok{0};std::vector<std::thread> workers;
  for(int j=0;j<8;++j)workers.emplace_back([&svc,&ok,j]{
    for(int i=0;i<15;++i) {
      auto q=j%2?ingress("POST",kBackendMissionStartPath):draft_request();
      const auto r=svc.dispatch_recovery_request(q,review());
      if((j%2 && r.status==423) || (!(j%2) && r.status==200)) ++ok;
      assert(!r.physical_command_emitted && !r.network_io_performed);
    }
  });
  for(auto& t:workers)t.join();
  assert(ok==120 && svc.snapshot().accepts_recovery_only);
}
static void service_shutdown_race() {
  TempDb db("h5r_shutdown_race");prepare(db);
  NativeOfflineRecoveryServiceLifecycleLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  std::atomic<bool> run{true};std::thread reader([&]{
    while(run.load()) {
      auto r=svc.dispatch_recovery_request(ingress("POST",kBackendMissionStopPath),review());
      assert(r.status==200 || r.status==423);
      assert(!r.physical_command_emitted && !r.network_io_performed);
    }
  });
  std::thread closer([&]{svc.shutdown();run.store(false);});
  closer.join();reader.join();
  assert(svc.snapshot().state==RecoveryServiceLifecycleLabState::permanently_closed);
  assert(svc.dispatch_recovery_request(draft_request(),review()).status==423);
  assert(!svc.admit_recovery_only());
}
// H5S: typed, independently-attested *offline* ingress handoff contract.
static RecoveryTrustedIngressEvidenceLab proof(std::uint64_t request_id=1,
                                               double received_sec=100.0) {
  RecoveryTrustedIngressEvidenceLab e;
  e.verified_transport_authenticated=true;
  e.verified_operator_session_id="SESSION_H4E_OPERATOR";
  e.verified_can_read_mission=true;
  e.verified_owns_control_lease=true;
  e.trusted_request_id=request_id;
  e.trusted_monotonic_received_sec=received_sec;
  return e;
}
static void ingress_h5s_boot_closed() {
  TempDb db("h5s_boot_closed");prepare(db);
  NativeOfflineRecoveryTrustedIngressLab svc(db.file.string(),review());
  assert(!svc.snapshot().admitted);
  const auto r=svc.dispatch(ingress("GET",kBackendMissionStatusPath),proof());
  assert(r.status==423 && !r.physical_command_emitted);
}
static void ingress_h5s_status() {
  TempDb db("h5s_status");prepare(db);
  NativeOfflineRecoveryTrustedIngressLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only() && !svc.admit_recovery_only());
  auto r=svc.dispatch(ingress("GET",kBackendMissionStatusPath),proof());
  assert(r.status==200);
  assert(r.body.find("\"recovery_quarantined\":true")!=std::string::npos);
  assert(svc.snapshot().last_accepted_request_id==1 && !r.network_io_performed);
}
static void ingress_h5s_forged_client() {
  TempDb db("h5s_forged_client");prepare(db);
  NativeOfflineRecoveryTrustedIngressLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  auto q=draft_request();
  q.http.principal.authenticated=false;
  q.http.principal.operator_session_id="CLIENT_FORGED";
  q.http.principal.can_read_mission=false;
  q.http.host_received_sec=-987.0;
  q.trusted_px4_disarmed=true;
  auto r=svc.dispatch(q,proof());
  assert(r.status==200 && r.body.find("\"preview_only\":true")!=std::string::npos);
  // Forged transport evidence, unlike raw client claims, must be rejected.
  auto e=proof(2,101);e.verified_operator_session_id="CLIENT_FORGED";
  assert(svc.dispatch(q,e).status==403);
}
static void ingress_h5s_unauth() {
  TempDb db("h5s_unauth");prepare(db);
  NativeOfflineRecoveryTrustedIngressLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  auto e=proof();e.verified_transport_authenticated=false;
  assert(svc.dispatch(ingress("POST",kBackendMissionStopPath),e).status==401);
  assert(svc.snapshot().last_accepted_request_id==0);
  assert(svc.dispatch(ingress("POST",kBackendMissionStopPath),proof()).status==200);
}
static void ingress_h5s_identity() {
  TempDb db("h5s_identity");prepare(db);
  NativeOfflineRecoveryTrustedIngressLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  auto e=proof();e.verified_operator_session_id="ANOTHER_SESSION";
  assert(svc.dispatch(draft_request(),e).status==403);
  e.verified_operator_session_id.clear();
  assert(svc.dispatch(draft_request(),e).status==403);
  e=proof();e.verified_can_read_mission=false;
  assert(svc.dispatch(draft_request(),e).status==403);
}
static void ingress_h5s_replay() {
  TempDb db("h5s_replay");prepare(db);
  NativeOfflineRecoveryTrustedIngressLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  assert(svc.dispatch(ingress("GET",kBackendMissionStatusPath),proof(1)).status==200);
  assert(svc.dispatch(ingress("GET",kBackendMissionStatusPath),proof(1)).status==409);
  assert(svc.dispatch(ingress("GET",kBackendMissionStatusPath),proof(0)).status==409);
  assert(svc.dispatch(ingress("POST",kBackendMissionStartPath),proof(2)).status==423);
  assert(svc.dispatch(ingress("GET",kBackendMissionStatusPath),proof(2)).status==409);
  assert(svc.snapshot().last_accepted_request_id==2);
}
static void ingress_h5s_clock() {
  TempDb db("h5s_clock");prepare(db);
  NativeOfflineRecoveryTrustedIngressLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  assert(svc.dispatch(ingress("GET",kBackendMissionStatusPath),proof(1,101)).status==200);
  auto e=proof(2,100);
  assert(svc.dispatch(ingress("GET",kBackendMissionStatusPath),e).status==400);
  e.trusted_monotonic_received_sec=std::numeric_limits<double>::infinity();
  assert(svc.dispatch(ingress("GET",kBackendMissionStatusPath),e).status==400);
  e.trusted_monotonic_received_sec=std::numeric_limits<double>::quiet_NaN();
  assert(svc.dispatch(ingress("GET",kBackendMissionStatusPath),e).status==400);
  e.trusted_monotonic_received_sec=101;
  assert(svc.dispatch(ingress("GET",kBackendMissionStatusPath),e).status==200);
}
static void ingress_h5s_limits() {
  TempDb db("h5s_limits");prepare(db);
  NativeOfflineRecoveryTrustedIngressLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  auto q=ingress("POST",kBackendMissionUploadPath);
  q.http.body=std::string(1025,'x');
  assert(svc.dispatch(q,proof()).status==413);
  assert(svc.snapshot().last_accepted_request_id==0);
  q=ingress("POST",kBackendMissionUploadPath);q.http.path=std::string(513,'x');
  assert(svc.dispatch(q,proof()).status==413);
  q=ingress("POST",kBackendMissionUploadPath);q.http.path.clear();
  assert(svc.dispatch(q,proof()).status==400);
  assert(svc.dispatch(ingress("POST",kBackendMissionUploadPath),proof()).status==423);
}
static void ingress_h5s_commands() {
  TempDb db("h5s_commands");prepare(db);
  NativeOfflineRecoveryTrustedIngressLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  std::uint64_t id=1;
  for(const char* path:{kBackendMissionStartPath,kBackendMissionPausePath,
      kBackendMissionResumePath,kBackendMissionNextPath,kBackendMissionSkipPath,
      kBackendMissionUploadPath,kBackendMissionUpdatePath,kBackendMissionLoadPath}) {
    auto q=ingress("POST",path);q.trusted_px4_disarmed=true;
    const auto r=svc.dispatch(q,proof(id++));
    assert(r.status==423 && !r.physical_command_emitted);
  }
  auto r=svc.dispatch(ingress("POST",kBackendMissionStopPath),proof(id++));
  assert(r.status==200 && r.body.find("\"safe_stop_intent\":true")!=std::string::npos);
  assert(!svc.snapshot().hardware_command_emitted);
}
static void ingress_h5s_preview() {
  TempDb db("h5s_preview");prepare(db);
  NativeOfflineRecoveryTrustedIngressLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  auto r=svc.dispatch(draft_request(),proof());
  assert(r.status==200 && r.body.find("\"preview_only\":true")!=std::string::npos);
  assert(r.body.find("marking_points")==std::string::npos);
  auto q=draft_request();q.http.path+="&override=1";
  assert(svc.dispatch(q,proof(2)).status==400);
}
static void ingress_h5s_tamper() {
  TempDb db("h5s_tamper");prepare(db);
  NativeOfflineRecoveryTrustedIngressLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  assert(svc.dispatch(draft_request(),proof()).status==200);
  sql(db,"UPDATE dyx_mission_recovery_staging SET last_sequence=999");
  assert(svc.dispatch(draft_request(),proof(2)).status==423);
  assert(svc.dispatch(ingress("POST",kBackendMissionStartPath),proof(3)).status==423);
}
static void ingress_h5s_shutdown() {
  TempDb db("h5s_shutdown");prepare(db);
  NativeOfflineRecoveryTrustedIngressLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  svc.shutdown();svc.shutdown();
  assert(svc.snapshot().closed_permanently && !svc.snapshot().admitted);
  assert(!svc.admit_recovery_only());
  assert(svc.dispatch(ingress("POST",kBackendMissionStopPath),proof()).status==423);
}
static void ingress_h5s_race() {
  TempDb db("h5s_race");prepare(db);
  NativeOfflineRecoveryTrustedIngressLab svc(db.file.string(),review());
  assert(svc.admit_recovery_only());
  std::atomic<std::uint64_t> seq{1};
  std::atomic<int> accepted{0};
  std::vector<std::thread> threads;
  for(int t=0;t<6;++t)threads.emplace_back([&] {
    for(int i=0;i<12;++i) {
      auto id=seq.fetch_add(1);
      auto r=svc.dispatch(ingress("GET",kBackendMissionStatusPath),proof(id));
      assert(r.status==200 || r.status==409);
      if(r.status==200)++accepted;
      assert(!r.physical_command_emitted && !r.network_io_performed);
    }
  });
  for(auto& th:threads)th.join();
  assert(accepted.load()>0 && svc.snapshot().admitted);
  svc.shutdown();
  assert(svc.dispatch(ingress("GET",kBackendMissionStatusPath),proof(1000)).status==423);
}
static void ingress_h5s_restart() {
  TempDb db("h5s_restart");prepare(db);
  for(int i=0;i<3;++i) {
    NativeOfflineRecoveryTrustedIngressLab svc(db.file.string(),review());
    assert(svc.dispatch(draft_request(),proof()).status==423);
    assert(svc.admit_recovery_only());
    assert(svc.dispatch(draft_request(),proof()).status==200);
    assert(svc.dispatch(ingress("POST",kBackendMissionStartPath),proof(2)).status==423);
    svc.shutdown();
  }
}

// H5T: offline host supervision; no actual process, listener, watchdog thread,
// PX4, motor, spray or physical STOP. Trusted facts are test-only contracts.
static RecoveryHostStartupEvidenceLab host_start(double now=100.0) {
  RecoveryHostStartupEvidenceLab e;
  e.independent_recovery_review_approved=true;
  e.independent_outputs_quiesced=true;
  e.verified_operator_session_id="SESSION_H4E_OPERATOR";
  e.trusted_monotonic_now_sec=now;
  return e;
}
static BackendHttpWireResponse host_get(NativeOfflineRecoveryHostSupervisorLab& host,
                                         std::uint64_t id=1, double rx=100.1,
                                         double now=100.2) {
  return host.dispatch(ingress("GET",kBackendMissionStatusPath),proof(id,rx),now);
}
static void host_boot_closed() {
  TempDb db("h5t_boot_closed");prepare(db);
  NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review());
  auto before=host.snapshot();
  assert(before.state==RecoveryHostSupervisorLabState::initial_closed);
  assert(!before.accepts_recovery_requests && !before.hardware_command_emitted);
  assert(host_get(host).status==423);
  assert(!host.observe_heartbeat(100.0));
  host.shutdown();
  assert(host.snapshot().state==RecoveryHostSupervisorLabState::permanently_closed);
}
static void host_bad_config() {
  TempDb db("h5t_bad_config");prepare(db);
  std::vector<RecoveryHostSupervisorLabConfig> invalid;
  auto c=RecoveryHostSupervisorLabConfig{};
  c.offline_no_listener_attached=false;invalid.push_back(c);
  c={};c.offline_no_dds_publisher_attached=false;invalid.push_back(c);
  c={};c.offline_no_actuator_attached=false;invalid.push_back(c);
  c={};c.max_heartbeat_gap_sec=0.0;invalid.push_back(c);
  c={};c.max_heartbeat_gap_sec=61.0;invalid.push_back(c);
  c={};c.max_heartbeat_gap_sec=std::numeric_limits<double>::quiet_NaN();
  invalid.push_back(c);
  for(const auto& config: invalid) {
    NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review(),config);
    assert(!host.start_recovery_only(host_start()));
    assert(host.snapshot().state==RecoveryHostSupervisorLabState::permanently_closed);
    assert(host_get(host).status==423);
    assert(!host.start_recovery_only(host_start()));
  }
}
static void host_bad_start() {
  TempDb db("h5t_bad_start");prepare(db);
  auto verify_failed=[&](const RecoveryHostStartupEvidenceLab& e) {
    NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review());
    assert(!host.start_recovery_only(e));
    assert(!host.start_recovery_only(host_start()));
    assert(host.snapshot().state==RecoveryHostSupervisorLabState::permanently_closed);
  };
  auto e=host_start();e.independent_recovery_review_approved=false;
  verify_failed(e);
  e=host_start();e.independent_outputs_quiesced=false;verify_failed(e);
  e=host_start();e.verified_operator_session_id="OTHER";verify_failed(e);
  e=host_start();e.trusted_monotonic_now_sec=-1;verify_failed(e);
  e=host_start();e.trusted_monotonic_now_sec=std::numeric_limits<double>::infinity();
  verify_failed(e);
}
static void host_admission() {
  TempDb db("h5t_admission");prepare(db);
  NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review());
  assert(host.start_recovery_only(host_start()));
  assert(!host.start_recovery_only(host_start()));
  auto r=host_get(host);
  assert(r.status==200 && r.body.find("\"recovery_quarantined\":true")!=std::string::npos);
  auto snap=host.snapshot();
  assert(snap.accepts_recovery_requests);
  assert(!snap.can_issue_motion && !snap.can_upload_or_load);
  assert(!snap.hardware_command_emitted && !snap.network_io_performed &&
         !snap.physical_stop_confirmed);
}
static void host_commands() {
  TempDb db("h5t_commands");prepare(db);
  NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review());
  assert(host.start_recovery_only(host_start()));
  std::uint64_t id=1;
  for(const auto* path:{kBackendMissionStartPath,kBackendMissionPausePath,
      kBackendMissionResumePath,kBackendMissionNextPath,kBackendMissionSkipPath,
      kBackendMissionUploadPath,kBackendMissionUpdatePath,kBackendMissionLoadPath}) {
    auto q=ingress("POST",path);q.trusted_px4_disarmed=true;
    auto r=host.dispatch(q,proof(id++,100.1),100.2);
    assert(r.status==423 && !r.physical_command_emitted && !r.network_io_performed);
  }
  const auto stop=host.dispatch(ingress("POST",kBackendMissionStopPath),
                                proof(id++,100.1),100.2);
  assert(stop.status==200 && stop.body.find("\"safe_stop_intent\":true")!=std::string::npos);
  assert(!host.snapshot().physical_stop_confirmed);
}
static void host_preview() {
  TempDb db("h5t_preview");prepare(db);
  NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review());
  assert(host.start_recovery_only(host_start()));
  auto r=host.dispatch(draft_request(),proof(1,100.1),100.2);
  assert(r.status==200 && r.body.find("\"preview_only\":true")!=std::string::npos);
  assert(r.body.find("marking_points")==std::string::npos);
  sql(db,"UPDATE dyx_mission_recovery_staging SET last_sequence=999");
  assert(host.dispatch(draft_request(),proof(2,100.2),100.3).status==423);
  assert(host.dispatch(ingress("POST",kBackendMissionStartPath),
                       proof(3,100.3),100.4).status==423);
}
static void host_identity() {
  TempDb db("h5t_identity");prepare(db);
  NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review());
  assert(host.start_recovery_only(host_start()));
  auto q=draft_request();q.trusted_px4_disarmed=true;
  q.http.principal.authenticated=false;
  q.http.principal.operator_session_id="CLIENT_FORGED";
  q.http.host_received_sec=-1000;
  assert(host.dispatch(q,proof(1,100.1),100.2).status==200);
  auto e=proof(2,100.2);e.verified_transport_authenticated=false;
  assert(host.dispatch(q,e,100.3).status==401);
  e=proof(2,100.2);e.verified_operator_session_id="OTHER";
  assert(host.dispatch(q,e,100.3).status==403);
  assert(host.snapshot().accepts_recovery_requests);
}
static void host_watchdog() {
  TempDb db("h5t_watchdog");prepare(db);
  RecoveryHostSupervisorLabConfig cfg;cfg.max_heartbeat_gap_sec=2.0;
  NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review(),cfg);
  assert(host.start_recovery_only(host_start()));
  assert(host_get(host,1,100.1,101.0).status==200);
  // No automatic thread: simulated host loop checks the deadline explicitly.
  assert(!host.observe_heartbeat(102.01));
  assert(host.snapshot().state==RecoveryHostSupervisorLabState::permanently_closed);
  assert(host_get(host,2,102.1,102.2).status==423);
  assert(!host.start_recovery_only(host_start()));
}
static void host_heartbeat() {
  TempDb db("h5t_heartbeat");prepare(db);
  RecoveryHostSupervisorLabConfig cfg;cfg.max_heartbeat_gap_sec=2.0;
  NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review(),cfg);
  assert(host.start_recovery_only(host_start()));
  for(int i=1;i<=18;++i)assert(host.observe_heartbeat(100.0+i));
  assert(host.dispatch(ingress("GET",kBackendMissionStatusPath),
                       proof(1,118.0),118.1).status==200);
  assert(host.snapshot().last_heartbeat_sec==118.0);
  assert(!host.observe_heartbeat(120.1));
  assert(host.snapshot().state==RecoveryHostSupervisorLabState::permanently_closed);
}
static void host_clock() {
  TempDb db("h5t_clock");prepare(db);
  NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review());
  assert(host.start_recovery_only(host_start()));
  assert(host.observe_heartbeat(101.0));
  assert(host_get(host,1,101.0,100.9).status==423);
  assert(host.snapshot().state==RecoveryHostSupervisorLabState::permanently_closed);
  NativeOfflineRecoveryHostSupervisorLab other(db.file.string(),review());
  assert(other.start_recovery_only(host_start()));
  assert(!other.observe_heartbeat(std::numeric_limits<double>::quiet_NaN()));
  assert(other.snapshot().state==RecoveryHostSupervisorLabState::permanently_closed);
}
static void host_receipt_time() {
  TempDb db("h5t_receipt");prepare(db);
  auto deny_case=[&](double received, double now) {
    NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review());
    assert(host.start_recovery_only(host_start()));
    assert(host.dispatch(draft_request(),proof(1,received),now).status==423);
    assert(host.snapshot().state==RecoveryHostSupervisorLabState::permanently_closed);
  };
  deny_case(99.0,100.1); // before boot
  deny_case(101.0,100.1); // claimed time in future
  deny_case(std::numeric_limits<double>::infinity(),100.1);
}
static void host_shutdown() {
  TempDb db("h5t_shutdown");prepare(db);
  NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review());
  assert(host.start_recovery_only(host_start()));
  host.shutdown();host.shutdown();
  assert(host.snapshot().state==RecoveryHostSupervisorLabState::permanently_closed);
  assert(host_get(host).status==423 && !host.observe_heartbeat(101));
  assert(!host.start_recovery_only(host_start()));
}
static void host_restart() {
  TempDb db("h5t_restart");prepare(db);
  for(int i=0;i<4;++i) {
    NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review());
    assert(host_get(host).status==423);
    assert(host.start_recovery_only(host_start()));
    assert(host.dispatch(draft_request(),proof(1,100.1),100.2).status==200);
    assert(host.dispatch(ingress("POST",kBackendMissionStartPath),
                         proof(2,100.2),100.3).status==423);
    host.shutdown();
  }
  // A new instance intentionally has a NEW in-memory request ID fence.
  // Cross-process durable replay protection is a separate unsolved need.
}
static void host_parallel() {
  TempDb db("h5t_parallel");prepare(db);
  NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review());
  assert(host.start_recovery_only(host_start()));
  std::atomic<std::uint64_t> ids{1};std::atomic<int> good{0};
  std::vector<std::thread> workers;
  for(int t=0;t<5;++t)workers.emplace_back([&] {
    for(int i=0;i<9;++i) {
      const auto id=ids.fetch_add(1);
      const auto r=host_get(host,id,100.1,100.2);
      assert(r.status==200 || r.status==409);
      assert(!r.physical_command_emitted && !r.network_io_performed);
      if(r.status==200)++good;
    }
  });
  for(auto& w:workers)w.join();
  assert(good.load()>0 && host.snapshot().accepts_recovery_requests);
}
static void host_shutdown_race() {
  TempDb db("h5t_shutdown_race");prepare(db);
  NativeOfflineRecoveryHostSupervisorLab host(db.file.string(),review());
  assert(host.start_recovery_only(host_start()));
  std::atomic<std::uint64_t> ids{1};
  std::vector<std::thread> workers;
  for(int t=0;t<4;++t)workers.emplace_back([&] {
    for(int i=0;i<10;++i) {
      const auto r=host_get(host,ids.fetch_add(1),100.1,100.2);
      assert(r.status==200 || r.status==409 || r.status==423);
      assert(!r.physical_command_emitted);
    }
  });
  workers.emplace_back([&]{host.shutdown();});
  for(auto& w:workers)w.join();
  assert(host.snapshot().state==RecoveryHostSupervisorLabState::permanently_closed);
  assert(host_get(host,1000).status==423);
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
  else if(m=="ingress_status")ingress_status();
  else if(m=="ingress_commands")ingress_commands();
  else if(m=="ingress_mutations")ingress_mutations();
  else if(m=="ingress_preview")ingress_preview();
  else if(m=="ingress_auth")ingress_auth();
  else if(m=="ingress_stop")ingress_stop();
  else if(m=="ingress_invalid")ingress_invalid();
  else if(m=="ingress_tamper")ingress_tamper();
  else if(m=="ingress_retired")ingress_retired();
  else if(m=="lifecycle_restart")lifecycle_restart();
  else if(m=="lifecycle_review")lifecycle_review();
  else if(m=="lifecycle_identity")lifecycle_identity();
  else if(m=="lifecycle_tamper")lifecycle_tamper();
  else if(m=="lifecycle_missing")lifecycle_missing();
  else if(m=="lifecycle_replays")lifecycle_replays();
  else if(m=="lifecycle_stop")lifecycle_stop();
  else if(m=="lifecycle_parallel")lifecycle_parallel();
  else if(m=="lifecycle_retire_race")lifecycle_retire_race();
  else if(m=="lifecycle_retire_restart")lifecycle_retire_restart();
  else if(m=="service_boot_closed")service_boot_closed();
  else if(m=="service_open_status")service_open_status();
  else if(m=="service_locked_actions")service_locked_actions();
  else if(m=="service_preview")service_preview();
  else if(m=="service_missing_db")service_missing_db();
  else if(m=="service_auth")service_auth();
  else if(m=="service_tamper")service_tamper();
  else if(m=="service_shutdown")service_shutdown();
  else if(m=="service_restart")service_restart();
  else if(m=="service_parallel")service_parallel();
  else if(m=="service_shutdown_race")service_shutdown_race();
  else if(m=="trusted_boot_closed")ingress_h5s_boot_closed();
  else if(m=="trusted_status")ingress_h5s_status();
  else if(m=="trusted_forged_client")ingress_h5s_forged_client();
  else if(m=="trusted_unauth")ingress_h5s_unauth();
  else if(m=="trusted_identity")ingress_h5s_identity();
  else if(m=="trusted_replay")ingress_h5s_replay();
  else if(m=="trusted_clock")ingress_h5s_clock();
  else if(m=="trusted_limits")ingress_h5s_limits();
  else if(m=="trusted_commands")ingress_h5s_commands();
  else if(m=="trusted_preview")ingress_h5s_preview();
  else if(m=="trusted_tamper")ingress_h5s_tamper();
  else if(m=="trusted_shutdown")ingress_h5s_shutdown();
  else if(m=="trusted_race")ingress_h5s_race();
  else if(m=="trusted_restart")ingress_h5s_restart();
  else if(m=="host_boot_closed")host_boot_closed();
  else if(m=="host_bad_config")host_bad_config();
  else if(m=="host_bad_start")host_bad_start();
  else if(m=="host_admission")host_admission();
  else if(m=="host_commands")host_commands();
  else if(m=="host_preview")host_preview();
  else if(m=="host_identity")host_identity();
  else if(m=="host_watchdog")host_watchdog();
  else if(m=="host_heartbeat")host_heartbeat();
  else if(m=="host_clock")host_clock();
  else if(m=="host_receipt_time")host_receipt_time();
  else if(m=="host_shutdown")host_shutdown();
  else if(m=="host_restart")host_restart();
  else if(m=="host_parallel")host_parallel();
  else if(m=="host_shutdown_race")host_shutdown_race();
  else return 2;
  std::cout<<"PASS H5O "<<m<<"\n";
}
