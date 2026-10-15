#pragma once
#include "backend_android_coordinator_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_ingress_lab.hpp"
#include <cassert>
using namespace dyx::rpp;

inline DraftIngressLabEvidence lab_proof(double server_sec,std::uint64_t nonce,
        const std::string& principal="SESSION_H4R_OPERATOR") {
  DraftIngressLabEvidence e;
  e.independently_verified_principal={true,true,true,principal};
  e.independently_verified_transport=true;
  e.independently_verified_session_epoch=3;
  e.independently_verified_ingress_nonce=nonce;
  e.independently_verified_auth_sec=server_sec-0.05;
  e.trusted_received_sec=server_sec;
  e.independently_verified_px4_source=true;
  e.px4_disarmed=true;
  e.independently_observed_px4_sec=server_sec-0.02;
  return e;
}
inline AndroidDraftSyncDecision lab_sync(NativeOfflineAndroidDraftCoordinator& app,
     NativeOfflineDraftIngressLab& lab,double start,std::uint64_t nonce) {
  auto request=app.begin_resync(start);
  assert(request.valid);
  const auto result=lab.snapshot(request,lab_proof(start+0.02,nonce));
  assert(result.trusted_reply_provenance && result.reached_backend &&
         !result.hardware_command_emitted && !result.network_io_performed);
  return app.complete_resync(request,
    draft_ingress_lab_sync_reply(request,result,start,start+0.04),start+0.05);
}
inline AndroidDraftCoordinatorResult lab_mutate(NativeOfflineAndroidDraftCoordinator& app,
      NativeOfflineDraftIngressLab& lab,AndroidEditorIntent intent,
      double start,std::uint64_t nonce) {
  const auto request=app.begin_mutation(intent,start);
  assert(request.valid);
  const auto result=lab.mutation(request,lab_proof(start+0.02,nonce));
  assert(result.trusted_reply_provenance && result.reached_backend &&
         !result.hardware_command_emitted && !result.network_io_performed);
  return app.complete_mutation(request,
    draft_ingress_lab_mutation_reply(request,result,start,start+0.04),start+0.05);
}
