#pragma once
#include "backend_android_draft_codec_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_android_draft_sync.hpp"
#include <cassert>
#include <string>
using namespace dyx::rpp;

inline AndroidEditorScreenContext sync_screen(const std::string& run="RUN_H4R_0001") {
  AndroidEditorScreenContext s;
  s.mission_run_id=run;
  s.ui_signed_in=true;
  s.ui_control_lease_visible=true;
  s.ui_network_connected=true;
  s.ui_px4_disarmed_indicator=true;
  s.unsaved_local_changes=true;
  s.marking_points={{0.0,0.0},{2.0,0.0},{2.0,3.0}};
  return s;
}
inline BackendTrustedUploadHttpRequest sync_trusted(const std::string& method,
      const std::string& path, const std::string& body, double now) {
  BackendTrustedUploadHttpRequest input;
  input.http.method=method;
  input.http.path=path;
  input.http.body=body;
  input.http.host_received_sec=now;
  input.http.principal={true,true,true,"SESSION_H4R_OPERATOR"};
  input.trusted_px4_disarmed=true;
  return input;
}
inline AndroidTrustedDraftSyncReply sync_verified(const AndroidDraftSyncRequest& request,
      const BackendHttpWireResponse& wire, double sent, double received) {
  AndroidTrustedDraftSyncReply reply;
  reply.wire=wire;
  reply.verified_authenticated_transport=true;
  reply.verified_request_id=request.request_id;
  reply.verified_request_path=request.path;
  reply.verified_sent_sec=sent;
  reply.verified_received_sec=received;
  return reply;
}
inline AndroidDraftSyncDecision sync_execute(NativeOfflineAndroidDraftSync& sync,
      NativeOfflineAndroidMissionEditor& editor, NativeBackendUploadHttpWire& server,
      const std::string& run, double sent, double received) {
  const auto request=sync.begin(run,sent);
  assert(request.valid && request.method=="GET" && request.body.empty());
  auto wire=server.handle(sync_trusted("GET",request.path,"",sent+0.02));
  return sync.complete(request,sync_verified(request,wire,sent,received),
                       received+0.01,editor);
}
inline void sync_no_hardware(const AndroidDraftSyncDecision& d) {
  assert(!d.network_io_performed && !d.hardware_command_emitted);
}
