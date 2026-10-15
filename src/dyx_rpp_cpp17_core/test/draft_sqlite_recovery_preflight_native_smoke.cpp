// DYX Autonomous H5F: no-recovery, no-actuation, strictly offline preflight.
#include "backend_upload_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_recovery_preflight_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_sqlite_store_lab.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
namespace fs=std::filesystem;
using namespace dyx::rpp;
struct H5fTempDb {
  fs::path dir,file;
  explicit H5fTempDb(const std::string& suffix) {
    const auto now=std::chrono::steady_clock::now().time_since_epoch().count();
    dir=fs::temp_directory_path()/("dyx_h5f_"+suffix+"_"+std::to_string(now));
    assert(fs::create_directory(dir));
    file=dir/"mission.sqlite";
    assert(NativeDraftSqliteStoreLab::provision_new(file.string(),
      {"SESSION_H4E_OPERATOR",3,0,0.0,1}));
    assert(NativeDraftSqliteMutationJournalLab::provision_schema_for_existing_store(file.string()));
  }
  ~H5fTempDb(){std::error_code ec;fs::remove_all(dir,ec);}
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
static BackendMissionUploadResponse upload(NativeOfflineDraftJournaledUploadLab& writer) {
  const auto r=upload_req(kBackendMissionUploadPath,1,10);
  const auto saved=writer.submit(proposal(r,1),r);
  assert(saved.journal_completed && saved.backend_reply.http_status==200);
  return saved.backend_reply;
}
static void seal(const fs::path& file) {
  NativeOfflineDraftPreviewCheckpointLab checkpoint(file.string());
  assert(checkpoint.seal_preview(trusted().saved_identity).state==
         DraftPreviewCheckpointLabState::sealed_preview);
}
static void nominal() {
  H5fTempDb db("nominal");
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(journal,backend);
  const auto first=upload(writer);
  const auto edit=update_req(first,2,11);
  const auto updated=writer.submit(proposal(edit,2),edit);
  assert(updated.journal_completed && updated.backend_reply.http_status==200);
  seal(db.file);
  NativeOfflineDraftRecoveryPreflightLab plan(db.file.string());
  const auto r=plan.inspect(trusted());
  assert(r.state==DraftRecoveryPreflightLabState::reviewed_preview_only &&
      r.last_nonce==2 && r.last_sequence==2 && r.last_received_sec==11 &&
      r.old_epoch==3 && r.proposed_new_epoch==4 && r.revision==2 &&
      r.signature==updated.backend_reply.signature &&
      r.reviewed_snapshot && r.reviewed_snapshot->marking_points.size()==2 &&
      !r.can_rehydrate_backend && !r.can_load && !r.can_start_or_resume &&
      !r.hardware_command_emitted && !gateway.emits_hardware_commands());
}
static void reopen() {
  H5fTempDb db("reopen");
  {
    NativeDraftSqliteMutationJournalLab journal(db.file.string());
    NativeMissionCommandGateway gateway;
    NativeBackendMissionUpload backend(gateway);
    NativeOfflineDraftJournaledUploadLab writer(journal,backend);
    upload(writer);seal(db.file);
  }
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload empty(gateway);
  NativeOfflineDraftRecoveryPreflightLab plan(db.file.string());
  const auto result=plan.inspect(trusted());
  assert(result.state==DraftRecoveryPreflightLabState::reviewed_preview_only &&
         result.last_sequence==1 && result.revision==1 &&
         !empty.has_draft() && !gateway.emits_hardware_commands());
}
static void authorization() {
  H5fTempDb db("authorization");
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(journal,backend);
  upload(writer);seal(db.file);
  NativeOfflineDraftRecoveryPreflightLab plan(db.file.string());
  auto r=trusted();r.saved_identity.authenticated=false;
  assert(plan.inspect(r).state==DraftRecoveryPreflightLabState::access_denied);
  r=trusted();r.saved_identity.operator_session_id="FORGED";
  assert(plan.inspect(r).state==DraftRecoveryPreflightLabState::access_denied);
  r=trusted();r.explicit_operator_review=false;
  assert(plan.inspect(r).state==DraftRecoveryPreflightLabState::operator_review_required);
  r=trusted();r.proposed_new_epoch=3;
  assert(plan.inspect(r).state==DraftRecoveryPreflightLabState::new_epoch_required);
  r=trusted();r.proposed_new_epoch=std::numeric_limits<std::uint64_t>::max();
  assert(plan.inspect(r).state==DraftRecoveryPreflightLabState::new_epoch_required);
  r=trusted();r.independently_disarmed=false;
  assert(plan.inspect(r).state==DraftRecoveryPreflightLabState::safety_evidence_required);
  r=trusted();r.independently_stationary=false;
  assert(plan.inspect(r).state==DraftRecoveryPreflightLabState::safety_evidence_required);
  r=trusted();r.independently_no_active_mission=false;
  assert(plan.inspect(r).state==DraftRecoveryPreflightLabState::safety_evidence_required);
  assert(!gateway.emits_hardware_commands());
}
static void stale() {
  H5fTempDb db("stale");
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(journal,backend);
  const auto first=upload(writer);seal(db.file);
  const auto update=update_req(first,2,11);
  assert(writer.submit(proposal(update,2),update).journal_completed);
  NativeOfflineDraftRecoveryPreflightLab plan(db.file.string());
  assert(plan.inspect(trusted()).state==DraftRecoveryPreflightLabState::checkpoint_unavailable);
}
static void pending() {
  H5fTempDb db("pending");
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(journal,backend);
  const auto first=upload(writer);seal(db.file);
  const auto update=update_req(first,2,11);
  assert(journal.begin(proposal(update,2)));
  NativeOfflineDraftRecoveryPreflightLab plan(db.file.string());
  assert(plan.inspect(trusted()).state==DraftRecoveryPreflightLabState::checkpoint_unavailable);
}
static void ready() {
  H5fTempDb db("ready");
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(journal,backend);
  const auto first=upload(writer);seal(db.file);
  auto load=load_req(first,2,11);
  assert(writer.submit(proposal(load,2),load).journal_completed);
  NativeOfflineDraftRecoveryPreflightLab plan(db.file.string());
  assert(plan.inspect(trusted()).state==DraftRecoveryPreflightLabState::checkpoint_unavailable);
  assert(!gateway.emits_hardware_commands());
}
static void no_checkpoint() {
  H5fTempDb db("no_checkpoint");
  NativeDraftSqliteMutationJournalLab journal(db.file.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab writer(journal,backend);
  upload(writer);
  NativeOfflineDraftRecoveryPreflightLab plan(db.file.string());
  assert(plan.inspect(trusted()).state==DraftRecoveryPreflightLabState::checkpoint_unavailable);
}
int main(int argc,char** argv) {
  assert(argc==2);
  const std::string mode=argv[1];
  if(mode=="nominal")nominal();
  else if(mode=="reopen")reopen();
  else if(mode=="authorization")authorization();
  else if(mode=="stale")stale();
  else if(mode=="pending")pending();
  else if(mode=="ready")ready();
  else if(mode=="no_checkpoint")no_checkpoint();
  else return 2;
  std::cout<<"PASS H5F non-actuating PREVIEW preflight "<<mode<<'\n';
}
