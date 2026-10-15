// Phase 2H5A: host-only integration of H4Z journal + actual mission backend.
// No network, physical PX4, DDS or actuator access.
#include "backend_upload_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_journaled_upload_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_sqlite_store_lab.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
namespace fs=std::filesystem;

struct H5aTempDb final {
  fs::path dir,db;
  explicit H5aTempDb(const char* mode) {
    const auto n=std::chrono::steady_clock::now().time_since_epoch().count();
    dir=fs::temp_directory_path()/(std::string("dyx_h5a_")+mode+"_"+std::to_string(n));
    assert(fs::create_directory(dir));
    db=dir/"mission.sqlite";
    assert(NativeDraftSqliteStoreLab::provision_new(db.string(),
        {"SESSION_H4E_OPERATOR",3,0,0.0,1}));
    assert(NativeDraftSqliteMutationJournalLab::provision_schema_for_existing_store(db.string()));
  }
  ~H5aTempDb() { std::error_code ec; fs::remove_all(dir,ec); }
};

static DraftJournalLabProposal h5a_proposal(const BackendMissionUploadRequest& request,
                                            std::uint64_t nonce) {
  DraftJournalLabProposal p;
  p.operator_session_id=request.principal.operator_session_id;
  p.session_epoch=3;
  p.ingress_nonce=nonce;
  p.received_sec=request.host_received_sec;
  p.mission_run_id=request.mission_run_id;
  p.sequence=request.sequence;
  p.route=request.path;
  assert(journaled_upload_lab_detail::json_request(request,p.canonical_request_json));
  return p;
}

static void nominal() {
  H5aTempDb db("nominal");
  NativeDraftSqliteMutationJournalLab journal(db.db.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab bridge(journal,backend);
  assert(!bridge.quarantined());
  bool history=true;
  assert(journal.has_history(history) && !history);
  auto u=upload_req(kBackendMissionUploadPath,1,10.0);
  const auto uploaded=bridge.submit(h5a_proposal(u,1),u);
  assert(uploaded.backend_invoked && uploaded.journal_reserved &&
         uploaded.journal_completed && !uploaded.requires_reconciliation &&
         uploaded.backend_reply.http_status==200 && backend.has_draft());
  DraftJournalLabEntry entry;
  assert(journal.inspect({"SESSION_H4E_OPERATOR",3,1},entry));
  assert(entry.status=="COMMITTED" && entry.decision.stage=="PREVIEW" &&
         entry.decision.complete_snapshot_json.find("navigation_points")!=std::string::npos);
  assert(journal.has_history(history) && history);
  auto update=update_req(uploaded.backend_reply,2,11.0);
  update.marking_points[1].x=3.0;
  const auto updated=bridge.submit(h5a_proposal(update,2),update);
  assert(updated.journal_completed && updated.backend_reply.http_status==200 &&
         updated.backend_reply.revision==2);
  auto load=load_req(updated.backend_reply,3,12.0);
  const auto loaded=bridge.submit(h5a_proposal(load,3),load);
  assert(loaded.journal_completed && loaded.backend_reply.loaded &&
         gateway.mission().stage()==MissionStage::ready && bridge.quarantined());
  assert(journal.inspect({"SESSION_H4E_OPERATOR",3,3},entry) &&
         entry.decision.stage=="READY" &&
         entry.decision.complete_snapshot_json.find("\"loaded\":true")!=std::string::npos);
  const auto blocked=bridge.submit(h5a_proposal(load,4),load);
  assert(!blocked.backend_invoked && blocked.requires_reconciliation);
  assert(!gateway.emits_hardware_commands());
}

static void binding() {
  H5aTempDb db("binding");
  NativeDraftSqliteMutationJournalLab journal(db.db.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab bridge(journal,backend);
  auto u=upload_req(kBackendMissionUploadPath,1,20.0);
  auto p=h5a_proposal(u,1);
  p.canonical_request_json="{\"sequence\":1}";
  assert(!bridge.submit(p,u).backend_invoked);
  p=h5a_proposal(u,1);
  p.operator_session_id="ANOTHER_OPERATOR";
  assert(!bridge.submit(p,u).backend_invoked);
  p=h5a_proposal(u,1);
  p.received_sec+=1.0;
  assert(!bridge.submit(p,u).backend_invoked);
  p=h5a_proposal(u,1);
  p.route=kBackendMissionLoadPath;
  assert(!bridge.submit(p,u).backend_invoked);
  p=h5a_proposal(u,1);
  auto stolen=u;
  stolen.principal.authenticated=false;
  assert(!bridge.submit(p,stolen).backend_invoked);
  stolen=u;
  stolen.trusted_px4_disarmed=false;
  assert(!bridge.submit(p,stolen).backend_invoked);
  bool any=true;
  assert(journal.has_history(any) && !any);
  assert(bridge.submit(h5a_proposal(u,1),u).journal_completed);
  assert(backend.has_draft() && !gateway.emits_hardware_commands());
}

static void reopen() {
  H5aTempDb db("reopen");
  {
    NativeDraftSqliteMutationJournalLab journal(db.db.string());
    NativeMissionCommandGateway gateway;
    NativeBackendMissionUpload backend(gateway);
    NativeOfflineDraftJournaledUploadLab bridge(journal,backend);
    auto u=upload_req(kBackendMissionUploadPath,1,30.0);
    assert(bridge.submit(h5a_proposal(u,1),u).journal_completed);
  }
  // New backend loses in-memory draft. This adapter MUST deny, even though
  // the journal contains a completed snapshot, until secure restore is built.
  NativeDraftSqliteMutationJournalLab reopened(db.db.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload empty_backend(gateway);
  NativeOfflineDraftJournaledUploadLab bridge(reopened,empty_backend);
  assert(bridge.quarantined());
  auto u=upload_req(kBackendMissionUploadPath,2,31.0);
  assert(!bridge.submit(h5a_proposal(u,2),u).backend_invoked);
  assert(!empty_backend.has_draft() && !gateway.emits_hardware_commands());
}

static void pending() {
  H5aTempDb db("pending");
  auto u=upload_req(kBackendMissionUploadPath,1,40.0);
  {
    NativeDraftSqliteMutationJournalLab journal(db.db.string());
    assert(journal.begin(h5a_proposal(u,1)));
    // Force a simulated process death before real backend invocation.
  }
  NativeDraftSqliteMutationJournalLab journal(db.db.string());
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineDraftJournaledUploadLab bridge(journal,backend);
  assert(bridge.quarantined());
  bool pending_record=false;
  assert(journal.has_pending(pending_record) && pending_record);
  assert(!bridge.submit(h5a_proposal(u,2),u).backend_invoked);
  assert(!backend.has_draft() && !gateway.emits_hardware_commands());
}
int main(int argc,char** argv) {
  assert(argc==2);
  std::string mode=argv[1];
  if (mode=="nominal") nominal();
  else if (mode=="binding") binding();
  else if (mode=="reopen") reopen();
  else if (mode=="pending") pending();
  else return 2;
  std::cout << "PASS H5A journal/backend " << mode << '\n';
}
