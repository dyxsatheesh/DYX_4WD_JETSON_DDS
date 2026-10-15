#pragma once
#include "dyx_rpp_cpp17_core/backend_android_draft_http_codec.hpp"
#include <cassert>
#include <string>
using namespace dyx::rpp;
inline AndroidEditorProposal codec_proposal(const std::string& path,
    std::uint64_t sequence, std::uint64_t revision=0,
    const std::string& signature={}) {
  AndroidEditorProposal p;
  p.prepared=true;
  p.method="POST";
  p.path=path;
  p.mission_run_id="RUN_H4Q_0001";
  p.sequence=sequence;
  p.expected_revision=revision;
  p.expected_signature=signature;
  if (path!=kBackendMissionLoadPath) {
    p.marking_points={{0.0,0.0},{2.5,0.0},{2.5,3.5}};
    p.extension_enabled=false;
    p.extension_distance_m=4.0;
    p.mission_mode=MissionMode::automatic;
  }
  return p;
}
inline BackendTrustedUploadHttpRequest codec_trusted_http(
    const AndroidDraftEncodedRequest& p,double when=10.0) {
  BackendTrustedUploadHttpRequest r;
  r.http.method=p.method;
  r.http.path=p.path;
  r.http.body=p.body;
  r.http.principal={true,true,true,"SESSION_H4Q_OPERATOR"};
  r.http.host_received_sec=when;
  r.trusted_px4_disarmed=true;
  return r;
}
inline AndroidTrustedDraftHttpReply codec_verified_response(
    const BackendHttpWireResponse& wire, const AndroidDraftEncodedRequest& request,
    double sent=10.0, double received=10.1) {
  AndroidTrustedDraftHttpReply r;
  r.wire=wire;
  r.verified_authenticated_transport=true;
  r.verified_request_sequence=request.sequence;
  r.verified_request_path=request.path;
  r.verified_sent_sec=sent;
  r.verified_received_sec=received;
  return r;
}
inline std::string codec_signature(NativeBackendMissionUpload& backend) {
  BackendMissionUploadRequest r;
  r.path=kBackendMissionDraftPath;
  r.method="GET";
  r.mission_run_id="RUN_H4Q_0001";
  r.principal={true,true,false,"SESSION_H4Q_OPERATOR"};
  const auto q=backend.handle(r);
  assert(q.accepted());
  assert(valid_path_signature(q.signature));
  return q.signature;
}
