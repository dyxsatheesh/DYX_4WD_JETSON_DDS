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
  assert(boundary_sync(app,boundary,source,40.0,1).editor_resynchronized);
  const auto request=app.begin_mutation(AndroidEditorIntent::upload,40.1);
  assert(request.valid);
  const auto raw=framed_lab_request(request.method,request.path,request.body);
  source.set_time_nonce(40.12,2);
  const auto ok=boundary.mutation(raw,request);
  assert(ok.reached_backend && ok.wire.status==200 && backend.has_draft());
  // Simulate reply lost: never infer POST failed; do not auto-retransmit.
  assert(!app.expire_mutation(42.2).verified_backend_decision);
  assert(app.requires_resync());
  source.set_time_nonce(42.22,2);
  assert(!boundary.mutation(raw,request).reached_backend); // nonce replay
  source.set_time_nonce(42.24,3);
  const auto repeated=boundary.mutation(raw,request);
  assert(repeated.reached_backend && repeated.wire.status==409);
  assert(!app.begin_mutation(AndroidEditorIntent::load,42.26).valid);
  assert(boundary_sync(app,boundary,source,42.3,4).draft.revision==1);
  assert(!app.requires_resync());
  assert(!gateway.has_active_mission() && !gateway.emits_hardware_commands());
}
