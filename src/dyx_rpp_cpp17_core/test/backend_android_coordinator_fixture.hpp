#pragma once
#include "backend_android_sync_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_android_draft_coordinator.hpp"
#include <cassert>
using namespace dyx::rpp;

inline AndroidDraftSyncDecision coordinate_sync(NativeOfflineAndroidDraftCoordinator& app,
     NativeBackendUploadHttpWire& server, double t,
     const std::string& principal="SESSION_H4Q_OPERATOR") {
  const auto request=app.begin_resync(t);
  assert(request.valid && request.method=="GET");
  auto input=sync_trusted(request.method,request.path,request.body,t+0.01);
  input.http.principal={true,true,true,principal};
  const auto wire=server.handle(input);
  return app.complete_resync(request,sync_verified(request,wire,t,t+0.03),t+0.04);
}
inline AndroidDraftCoordinatorResult coordinate_mutate(
     NativeOfflineAndroidDraftCoordinator& app,
     NativeBackendUploadHttpWire& server, AndroidEditorIntent intent,double t) {
  const auto request=app.begin_mutation(intent,t);
  assert(request.valid && request.method=="POST" && !request.body.empty());
  const auto wire=server.handle(codec_trusted_http(request,t+0.01));
  return app.complete_mutation(request,
      codec_verified_response(wire,request,t,t+0.03),t+0.04);
}
inline void coordinate_safe(const AndroidDraftCoordinatorResult& r) {
  assert(!r.hardware_command_emitted && !r.network_io_performed);
}
