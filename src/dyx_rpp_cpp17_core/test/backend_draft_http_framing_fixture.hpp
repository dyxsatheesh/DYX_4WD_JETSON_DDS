#pragma once
#include "backend_draft_ingress_lab_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_http_framing_lab.hpp"
#include <cassert>
#include <string>
using namespace dyx::rpp;

// Canonical laboratory-only request builder. No TCP/HTTP traffic is sent.
inline std::string framed_lab_request(const std::string& method,
    const std::string& path,const std::string& body) {
  std::string frame=method+" "+path+" HTTP/1.1\r\nHost: offline.invalid\r\n";
  if (method=="POST") frame+="Content-Type: application/json\r\n";
  frame+="Content-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
  return frame;
}
inline AndroidDraftSyncDecision framed_sync(NativeOfflineAndroidDraftCoordinator& app,
     NativeOfflineDraftHttpFramingLab& framing,double start,std::uint64_t nonce) {
  const auto request=app.begin_resync(start);
  assert(request.valid);
  const auto received=framing.snapshot(
      framed_lab_request(request.method,request.path,request.body),
      request,lab_proof(start+0.02,nonce));
  assert(received.reached_backend && received.trusted_reply_provenance &&
         !received.network_io_performed && !received.hardware_command_emitted);
  return app.complete_resync(request,
      draft_ingress_lab_sync_reply(request,received,start,start+0.04),start+0.05);
}
inline AndroidDraftCoordinatorResult framed_mutation(
    NativeOfflineAndroidDraftCoordinator& app,
    NativeOfflineDraftHttpFramingLab& framing,AndroidEditorIntent intent,
    double start,std::uint64_t nonce) {
  const auto request=app.begin_mutation(intent,start);
  assert(request.valid);
  const auto received=framing.mutation(
      framed_lab_request(request.method,request.path,request.body),
      request,lab_proof(start+0.02,nonce));
  assert(received.reached_backend && received.trusted_reply_provenance &&
         !received.network_io_performed && !received.hardware_command_emitted);
  return app.complete_mutation(request,
      draft_ingress_lab_mutation_reply(request,received,start,start+0.04),start+0.05);
}
