#include "backend_draft_security_boundary_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  NativeOfflineDraftIngressLab ingress(wire);
  NativeOfflineDraftHttpFramingLab framing(ingress);
  BoundaryLabFixtureSource source;
  NativeOfflineDraftSecurityBoundaryLab boundary(framing,source);
  NativeOfflineAndroidDraftCoordinator app;
  app.show(sync_screen());
  assert(boundary_sync(app,boundary,source,20.0,1).editor_resynchronized);
  const auto req=app.begin_mutation(AndroidEditorIntent::upload,20.1);
  assert(req.valid);
  source.set_time_nonce(20.12,2);
  const auto raw=framed_lab_request(req.method,req.path,req.body);
  const int clock_reads=source.clock_calls, sessions=source.session_calls;
  const int px4_reads=source.px4_calls;
  auto reject=[&](const std::string& frame,const AndroidDraftEncodedRequest& proposal) {
    const auto denied=boundary.mutation(frame,proposal);
    assert(!denied.reached_backend && !denied.trusted_reply_provenance &&
           denied.wire.status==400 && !denied.network_io_performed &&
           !denied.hardware_command_emitted);
    assert(source.clock_calls==clock_reads && source.session_calls==sessions &&
           source.px4_calls==px4_reads);
    assert(ingress.last_ingress_nonce()==1 && !backend.has_draft());
  };
  auto altered=raw;
  altered.insert(altered.find("\r\n\r\n"),"\r\nX-Verified-PX4: disarmed");
  reject(altered,req);
  altered=raw;
  altered.insert(altered.find("\r\n\r\n"),"\r\nAuthorization: Bearer fake");
  reject(altered,req);
  altered=raw;
  altered.insert(altered.find("\r\n\r\n"),"\r\nContent-Length: 0");
  reject(altered,req);
  altered=raw+raw;
  reject(altered,req);
  auto forged=req;
  forged.body+=" ";
  reject(raw,forged);
  forged=req;
  forged.path=kBackendMissionLoadPath;
  reject(raw,forged);
  const auto ok=boundary.mutation(raw,req);
  assert(ok.reached_backend && ok.wire.status==200 && backend.has_draft());
  assert(source.px4_calls==px4_reads+1);
  assert(!gateway.emits_hardware_commands());
}
