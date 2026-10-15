// DYX Autonomous H5E — OFFLINE versioned PREVIEW checkpoint smoke tests.
// The checkpoint is for read-only post-restart operator map review ONLY.
#include "backend_upload_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_preview_checkpoint_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_sqlite_store_lab.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
namespace fs=std::filesystem;
using namespace dyx::rpp;
struct H5eTempDb {
  fs::path dir,file;
  explicit H5eTempDb(const std::string& suffix) {
    const auto n=std::chrono::steady_clock::now().time_since_epoch().count();
    dir=fs::temp_directory_path()/("dyx_h5e_"+suffix+"_"+std::to_string(n));
    assert(fs::create_directory(dir));
    file=dir/"missions.sqlite";
    assert(NativeDraftSqliteStoreLab::provision_new(file.string(),
       {"SESSION_H4E_OPERATOR",3,0,0.0,1}));
    assert(NativeDraftSqliteMutationJournalLab::provision_schema_for_existing_store(file.string()));
  }
  ~H5eTempDb(){std::error_code e;fs::remove_all(dir,e);}
};
static DraftReconciliationLabIdentity identity() {
  return {true,true,true,"SESSION_H4E_OPERATOR",3};
}
static DraftJournalLabProposal proposal(const BackendMissionUploadRequest& r,std::uint64_t nonce) {
  DraftJournalLabProposal p;
  p.operator_session_id=r.principal.operator_session_id;
  p.session_epoch=3;p.ingress_nonce=nonce;p.received_sec=r.host_received_sec;
  p.mission_run_id=r.mission_run_id;p.sequence=r.sequence;p.route=r.path;
  assert(journaled_upload_lab_detail::json_request(r,p.canonical_request_json));
  return p;
}
static BackendMissionUploadResponse upload_once(const fs::path& db,
      NativeMissionCommandGateway& gateway,NativeBackendMissionUpload& backend) {
  NativeDraftSqliteMutationJournalLab j(db.string());
  NativeOfflineDraftJournaledUploadLab writer(j,backend);
  const auto u=upload_req(kBackendMissionUploadPath,1,10);
  const auto result=writer.submit(proposal(u,1),u);
  assert(result.journal_completed && result.backend_reply.http_status==200);
  assert(!gateway.emits_hardware_commands());
  return result.backend_reply;
}
static void nominal() {
  H5eTempDb temp("nominal");
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  upload_once(temp.file,gateway,backend);
  NativeOfflineDraftPreviewCheckpointLab cp(temp.file.string());
  const auto sealed=cp.seal_preview(identity());
  assert(sealed.state==DraftPreviewCheckpointLabState::sealed_preview);
  assert(sealed.schema_version==1 && sealed.ingress_nonce==1 && sealed.revision==1);
  const auto review=cp.inspect_preview(identity());
  assert(review.state==DraftPreviewCheckpointLabState::verified_preview &&
         review.review_snapshot && review.review_snapshot->marking_points.size()==2 &&
         review.signature==sealed.signature &&
         !review.can_restore && !review.can_load && !review.can_start_or_resume &&
         !review.hardware_command_emitted && !gateway.emits_hardware_commands());
  // Re-seal without mutation is idempotent.
  assert(cp.seal_preview(identity()).state==DraftPreviewCheckpointLabState::sealed_preview);
}
static void reopen() {
  H5eTempDb temp("reopen");
  {
    NativeMissionCommandGateway gateway;
    NativeBackendMissionUpload backend(gateway);
    upload_once(temp.file,gateway,backend);
    NativeOfflineDraftPreviewCheckpointLab cp(temp.file.string());
    assert(cp.seal_preview(identity()).state==DraftPreviewCheckpointLabState::sealed_preview);
  }
  // No backend memory is recovered: only an independently inspectable copy.
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload fresh(gateway);
  NativeOfflineDraftPreviewCheckpointLab reopened(temp.file.string());
  const auto result=reopened.inspect_preview(identity());
  assert(result.state==DraftPreviewCheckpointLabState::verified_preview &&
         result.review_snapshot && !fresh.has_draft() &&
         !gateway.emits_hardware_commands() && !result.can_restore);
}
static void stale() {
  H5eTempDb temp("stale");
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeDraftSqliteMutationJournalLab j(temp.file.string());
  NativeOfflineDraftJournaledUploadLab writer(j,backend);
  auto upload=upload_req(kBackendMissionUploadPath,1,10);
  const auto initial=writer.submit(proposal(upload,1),upload);
  assert(initial.journal_completed);
  NativeOfflineDraftPreviewCheckpointLab cp(temp.file.string());
  assert(cp.seal_preview(identity()).state==DraftPreviewCheckpointLabState::sealed_preview);
  auto update=update_req(initial.backend_reply,2,11);
  update.marking_points[1].x=3.0;
  assert(writer.submit(proposal(update,2),update).journal_completed);
  // This checkpoint is stale and may NOT be silently overwritten.
  assert(cp.seal_preview(identity()).state==DraftPreviewCheckpointLabState::checkpoint_mismatch);
  assert(cp.inspect_preview(identity()).state==DraftPreviewCheckpointLabState::checkpoint_mismatch);
  assert(!gateway.emits_hardware_commands());
}
static void ready() {
  H5eTempDb temp("ready");
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeDraftSqliteMutationJournalLab j(temp.file.string());
  NativeOfflineDraftJournaledUploadLab writer(j,backend);
  auto upload=upload_req(kBackendMissionUploadPath,1,10);
  const auto initial=writer.submit(proposal(upload,1),upload);
  assert(initial.journal_completed);
  NativeOfflineDraftPreviewCheckpointLab cp(temp.file.string());
  assert(cp.seal_preview(identity()).state==DraftPreviewCheckpointLabState::sealed_preview);
  const auto load=load_req(initial.backend_reply,2,11);
  assert(writer.submit(proposal(load,2),load).journal_completed);
  assert(cp.inspect_preview(identity()).state==DraftPreviewCheckpointLabState::rejected_history);
  assert(cp.seal_preview(identity()).state==DraftPreviewCheckpointLabState::rejected_history);
  assert(gateway.mission().stage()==MissionStage::ready && !gateway.emits_hardware_commands());
}
static void pending() {
  H5eTempDb temp("pending");
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeDraftSqliteMutationJournalLab j(temp.file.string());
  NativeOfflineDraftJournaledUploadLab writer(j,backend);
  auto upload=upload_req(kBackendMissionUploadPath,1,10);
  const auto initial=writer.submit(proposal(upload,1),upload);
  assert(initial.journal_completed);
  NativeOfflineDraftPreviewCheckpointLab cp(temp.file.string());
  assert(cp.seal_preview(identity()).state==DraftPreviewCheckpointLabState::sealed_preview);
  const auto upd=update_req(initial.backend_reply,2,11);
  assert(j.begin(proposal(upd,2)));
  assert(cp.inspect_preview(identity()).state==DraftPreviewCheckpointLabState::rejected_history);
  assert(cp.seal_preview(identity()).state==DraftPreviewCheckpointLabState::rejected_history);
}
static void tamper() {
  H5eTempDb temp("tamper");
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  upload_once(temp.file,gateway,backend);
  NativeOfflineDraftPreviewCheckpointLab cp(temp.file.string());
  assert(cp.seal_preview(identity()).state==DraftPreviewCheckpointLabState::sealed_preview);
  sqlite3* db=nullptr;
  assert(sqlite3_open_v2(temp.file.string().c_str(),&db,SQLITE_OPEN_READWRITE,nullptr)==SQLITE_OK);
  assert(sqlite3_exec(db,"UPDATE dyx_preview_checkpoint SET revision=99 WHERE singleton=1",
                      nullptr,nullptr,nullptr)==SQLITE_OK);
  assert(sqlite3_close_v2(db)==SQLITE_OK);
  assert(cp.inspect_preview(identity()).state==DraftPreviewCheckpointLabState::checkpoint_mismatch);
  // A client-faked principal must not read even a valid checkpoint.
  auto forged=identity();forged.authenticated=false;
  assert(cp.inspect_preview(forged).state==DraftPreviewCheckpointLabState::access_denied);
  forged=identity();forged.operator_session_id="UNTRUSTED_OPERATOR";
  assert(cp.inspect_preview(forged).state==DraftPreviewCheckpointLabState::rejected_history);
  assert(!gateway.emits_hardware_commands());
}
int main(int argc,char**argv) {
  assert(argc==2);
  const std::string mode=argv[1];
  if(mode=="nominal")nominal();
  else if(mode=="reopen")reopen();
  else if(mode=="stale")stale();
  else if(mode=="ready")ready();
  else if(mode=="pending")pending();
  else if(mode=="tamper")tamper();
  else return 2;
  std::cout<<"PASS H5E offline preview checkpoint "<<mode<<'\n';
}
