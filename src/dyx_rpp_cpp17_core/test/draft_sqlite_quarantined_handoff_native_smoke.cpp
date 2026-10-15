// DYX Autonomous H5I: persistent locked PREVIEW-only handoff. No actuation.
#include "backend_upload_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_quarantined_handoff_lab.hpp"
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
    auto count=std::chrono::steady_clock::now().time_since_epoch().count();
    dir=fs::temp_directory_path()/("dyx_h5i_"+mode+"_"+std::to_string(count));
    assert(fs::create_directory(dir));file=dir/"draft.sqlite";
    assert(NativeDraftSqliteStoreLab::provision_new(file.string(),
         {"SESSION_H4E_OPERATOR",3,0,0.0,1}));
    assert(NativeDraftSqliteMutationJournalLab::provision_schema_for_existing_store(file.string()));
  }
  ~TempDb(){std::error_code ec;fs::remove_all(dir,ec);}
};
static DraftRecoveryPreflightLabRequest initial_request() {
  DraftRecoveryPreflightLabRequest r;
  r.saved_identity={true,true,true,"SESSION_H4E_OPERATOR",3};
  r.proposed_new_epoch=4;
  r.explicit_operator_review=true;r.independently_disarmed=true;
  r.independently_stationary=true;r.independently_no_active_mission=true;
  return r;
}
static DraftQuarantinedHandoffLabRequest handoff_request() {
  DraftQuarantinedHandoffLabRequest r;
  r.current_identity={true,true,true,"SESSION_H4E_OPERATOR",4};
  r.explicit_operator_review=true;r.independently_disarmed=true;
  r.independently_stationary=true;r.independently_no_active_mission=true;
  return r;
}
static DraftJournalLabProposal proposal(const BackendMissionUploadRequest& r,
                                        std::uint64_t nonce) {
  DraftJournalLabProposal p;
  p.operator_session_id=r.principal.operator_session_id;p.session_epoch=3;
  p.ingress_nonce=nonce;p.received_sec=r.host_received_sec;
  p.mission_run_id=r.mission_run_id;p.sequence=r.sequence;p.route=r.path;
  assert(journaled_upload_lab_detail::json_request(r,p.canonical_request_json));
  return p;
}
static void upload_and_seal(TempDb& db) {
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  NativeOfflineDraftJournaledUploadLab w(journal,b);
  const auto r=upload_req(kBackendMissionUploadPath,1,10);
  const auto saved=w.submit(proposal(r,1),r);
  assert(saved.journal_completed && saved.backend_reply.http_status==200);
  NativeOfflineDraftPreviewCheckpointLab c(db.file.string());
  assert(c.seal_preview(initial_request().saved_identity).state==
         DraftPreviewCheckpointLabState::sealed_preview);
}
static void quarantine(TempDb& db) {
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  assert(NativeOfflineDraftEpochQuarantineLab(db.file.string())
           .quarantine(initial_request(),g,b).state==
         DraftEpochQuarantineLabState::quarantine_committed);
}
static DraftQuarantinedHandoffLabResult stage(TempDb& db) {
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  return NativeOfflineQuarantinedPreviewHandoffLab(db.file.string())
       .stage_for_review(handoff_request(),g,b);
}
static int stage_count(const TempDb& db) {
  sqlite3* sql=draft_journal_lab_detail::open_existing(db.file.string());
  draft_journal_lab_detail::Statement st(sql,"SELECT count(*) FROM dyx_mission_recovery_staging");
  assert(st && sqlite3_step(st.st)==SQLITE_ROW);
  const int count=sqlite3_column_int(st.st,0);
  sqlite3_finalize(st.st);st.st=nullptr;sqlite3_close_v2(sql);
  return count;
}
static void sql_exec(TempDb& db,const char* statement) {
  sqlite3* sql=draft_journal_lab_detail::open_existing(db.file.string());
  assert(draft_journal_lab_detail::exec(sql,statement));sqlite3_close_v2(sql);
}
static void nominal() {
  TempDb db("nominal");upload_and_seal(db);quarantine(db);
  auto r=stage(db);
  assert(r.state==DraftQuarantinedHandoffLabState::staged_preview &&
      r.old_epoch==3 && r.quarantined_epoch==4 && r.nonce==1 &&
      r.generation==3 && r.revision==1 && r.last_sequence==1 &&
      r.read_only_preview && r.read_only_preview->marking_points.size()==2 &&
      !r.can_unquarantine && !r.can_post && !r.can_load &&
      !r.can_start_or_resume && !r.hardware_command_emitted &&
      stage_count(db)==1);
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  assert(!journal.begin(proposal(upload_req(kBackendMissionUploadPath,2,20),2)));
}
static void restart() {
  TempDb db("restart");upload_and_seal(db);quarantine(db);
  assert(stage(db).state==DraftQuarantinedHandoffLabState::staged_preview);
  assert(stage(db).state==DraftQuarantinedHandoffLabState::already_staged);
  assert(stage_count(db)==1);
  NativeDraftSqliteMutationJournalLab reopened(db.file.string());
  auto r=upload_req(kBackendMissionUploadPath,2,20);
  auto p=proposal(r,2);p.session_epoch=4;
  assert(!reopened.begin(p));
}
static void auth() {
  TempDb db("auth");upload_and_seal(db);quarantine(db);
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  NativeOfflineQuarantinedPreviewHandoffLab h(db.file.string());
  auto r=handoff_request();r.current_identity.authenticated=false;
  assert(h.stage_for_review(r,g,b).state==DraftQuarantinedHandoffLabState::denied);
  r=handoff_request();r.current_identity.session_epoch=3;
  assert(h.stage_for_review(r,g,b).state==DraftQuarantinedHandoffLabState::inconsistent_barrier);
  r=handoff_request();r.current_identity.operator_session_id="FORGED";
  assert(h.stage_for_review(r,g,b).state==DraftQuarantinedHandoffLabState::inconsistent_barrier);
  r=handoff_request();r.independently_stationary=false;
  assert(h.stage_for_review(r,g,b).state==DraftQuarantinedHandoffLabState::denied);
  r=handoff_request();r.independently_disarmed=false;
  assert(h.stage_for_review(r,g,b).state==DraftQuarantinedHandoffLabState::denied);
  r=handoff_request();r.explicit_operator_review=false;
  assert(h.stage_for_review(r,g,b).state==DraftQuarantinedHandoffLabState::denied);
  assert(stage_count(db)==0);
}
static void missing() {
  TempDb db("missing");upload_and_seal(db);
  assert(stage(db).state==DraftQuarantinedHandoffLabState::not_quarantined &&
         stage_count(db)==0);
}
static void tamper() {
  TempDb db("tamper");upload_and_seal(db);quarantine(db);
  sql_exec(db,"UPDATE dyx_preview_checkpoint SET signature='INVALID'");
  assert(stage(db).state==DraftQuarantinedHandoffLabState::snapshot_rejected);
  assert(stage_count(db)==0);
}
static void history() {
  TempDb db("history");upload_and_seal(db);quarantine(db);
  sql_exec(db,"UPDATE dyx_mission_mutations SET request_json='{}' WHERE nonce=1");
  assert(stage(db).state==DraftQuarantinedHandoffLabState::history_rejected);
  assert(stage_count(db)==0);
}
static void barrier() {
  TempDb db("barrier");upload_and_seal(db);quarantine(db);
  sql_exec(db,"UPDATE dyx_mission_recovery_epoch_barrier SET nonce=2");
  assert(stage(db).state==DraftQuarantinedHandoffLabState::inconsistent_barrier);
  assert(stage_count(db)==0);
}
static void failure() {
  TempDb db("failure");upload_and_seal(db);quarantine(db);
  sql_exec(db,"CREATE TRIGGER fail_h5i BEFORE INSERT ON dyx_mission_recovery_staging "
              "BEGIN SELECT RAISE(ABORT,'injected'); END");
  assert(stage(db).state==DraftQuarantinedHandoffLabState::write_failure);
  assert(stage_count(db)==0);
  sql_exec(db,"DROP TRIGGER fail_h5i");
  assert(stage(db).state==DraftQuarantinedHandoffLabState::staged_preview);
}
static void conflict() {
  TempDb db("conflict");upload_and_seal(db);quarantine(db);
  assert(stage(db).state==DraftQuarantinedHandoffLabState::staged_preview);
  sql_exec(db,"UPDATE dyx_mission_recovery_staging SET last_sequence=99");
  assert(stage(db).state==DraftQuarantinedHandoffLabState::staging_conflict);
}
static void active() {
  TempDb db("active");upload_and_seal(db);quarantine(db);
  NativeMissionCommandGateway g;NativeBackendMissionUpload b(g);
  auto r=upload_req(kBackendMissionUploadPath,1,10);
  assert(b.handle(r).http_status==200 && b.has_draft());
  assert(NativeOfflineQuarantinedPreviewHandoffLab(db.file.string())
    .stage_for_review(handoff_request(),g,b).state==
    DraftQuarantinedHandoffLabState::backend_active);
  assert(stage_count(db)==0);
}
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  const std::string mode=argv[1];
  if(mode=="nominal")nominal();
  else if(mode=="restart")restart();
  else if(mode=="auth")auth();
  else if(mode=="missing")missing();
  else if(mode=="tamper")tamper();
  else if(mode=="history")history();
  else if(mode=="barrier")barrier();
  else if(mode=="failure")failure();
  else if(mode=="conflict")conflict();
  else if(mode=="active")active();
  else return 2;
  std::cout<<"PASS H5I "<<mode<<"\n";
}
