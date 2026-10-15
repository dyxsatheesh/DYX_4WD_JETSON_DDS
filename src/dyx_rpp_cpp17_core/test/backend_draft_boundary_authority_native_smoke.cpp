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
  assert(boundary_sync(app,boundary,source,30.0,1).editor_resynchronized);
  const auto req=app.begin_mutation(AndroidEditorIntent::upload,30.1);
  assert(req.valid);
  const auto raw=framed_lab_request(req.method,req.path,req.body);
  auto attempt=[&]() { return boundary.mutation(raw,req); };
  source.set_time_nonce(30.12,2);
  source.session_available=false;
  assert(attempt().wire.status==503);
  source.session_available=true;
  source.clock_available=false;
  assert(attempt().wire.status==503);
  source.clock_available=true;
  source.px4_available=false;
  assert(attempt().wire.status==503);
  source.px4_available=true;
  source.session.verified_transport=false;
  assert(!attempt().reached_backend);
  source.set_time_nonce(30.12,2);
  source.session.principal.owns_control_lease=false;
  assert(!attempt().reached_backend);
  source.set_time_nonce(30.12,2);
  source.session.verified_auth_sec=20.0;
  assert(!attempt().reached_backend);
  source.set_time_nonce(30.12,2);
  source.px4.verified_px4_source=false;
  assert(!attempt().reached_backend);
  source.set_time_nonce(30.12,2);
  source.px4.disarmed=false;
  assert(!attempt().reached_backend);
  source.set_time_nonce(30.12,2);
  source.px4.observed_sec=29.0;
  assert(!attempt().reached_backend);
  source.set_time_nonce(30.12,2);
  source.session.session_epoch=4;
  assert(!attempt().reached_backend);
  assert(ingress.last_ingress_nonce()==1 && !backend.has_draft());
  source.set_time_nonce(30.12,2);
  const auto ok=attempt();
  assert(ok.reached_backend && ok.wire.status==200 && backend.has_draft());
  assert(ingress.last_ingress_nonce()==2);
  assert(!attempt().reached_backend); // same verified nonce replay
  source.set_time_nonce(30.0,3); // host-monotonic regression latches
  assert(!attempt().reached_backend && ingress.fault_latched());
  source.set_time_nonce(31.0,4);
  assert(!attempt().reached_backend);
  assert(!gateway.has_active_mission() && !gateway.emits_hardware_commands());
}
