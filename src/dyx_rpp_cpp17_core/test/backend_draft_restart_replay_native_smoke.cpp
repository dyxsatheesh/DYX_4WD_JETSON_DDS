#include "backend_draft_restart_fence_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  RestartLabMemoryStore store;
  {
    NativeOfflineDraftIngressLab ingress(wire);
    NativeOfflineDraftHttpFramingLab framing(ingress);
    BoundaryLabFixtureSource source;
    NativeOfflineDraftRestartFenceSource fence(source,store);
    NativeOfflineDraftSecurityBoundaryLab boundary(framing,fence);
    NativeOfflineAndroidDraftCoordinator app;
    app.show(sync_screen());
    assert(restart_sync(app,boundary,source,20.0,1).editor_resynchronized);
    assert(restart_mutate(app,boundary,source,AndroidEditorIntent::upload,20.1,2)
           .verified_backend_decision);
    assert(store.record.last_ingress_nonce==2 && backend.has_draft());
  } // Jetson process-local ingress and source are reconstructed below.
  {
    NativeOfflineDraftIngressLab ingress(wire);
    NativeOfflineDraftHttpFramingLab framing(ingress);
    BoundaryLabFixtureSource source;
    NativeOfflineDraftRestartFenceSource fence(source,store);
    NativeOfflineDraftSecurityBoundaryLab boundary(framing,fence);
    NativeOfflineAndroidDraftCoordinator app;
    app.show(sync_screen());
    const auto get=app.begin_resync(20.2);
    assert(get.valid);
    const auto frame=framed_lab_request(get.method,get.path,get.body);
    source.set_time_nonce(20.22,2); // Used BEFORE simulated restart.
    auto denied=boundary.snapshot(frame,get);
    assert(!denied.reached_backend && denied.wire.status==503);
    assert(ingress.last_ingress_nonce()==0 && store.record.last_ingress_nonce==2);
    source.set_time_nonce(20.23,3);
    denied=boundary.snapshot(frame,get);
    assert(denied.reached_backend && denied.wire.status==200);
    assert(app.complete_resync(get,
        draft_ingress_lab_sync_reply(get,denied,20.2,20.24),20.25)
           .draft.revision==1);
    auto next=app.begin_resync(20.3);
    assert(next.valid);
    source.set_time_nonce(20.32,4);
    source.session.session_epoch=4;
    denied=boundary.snapshot(framed_lab_request(next.method,next.path,next.body),next);
    assert(!denied.reached_backend && denied.wire.status==503);
    source.set_time_nonce(20.32,4);
    source.session.principal.operator_session_id="ANOTHER_OPERATOR";
    denied=boundary.snapshot(framed_lab_request(next.method,next.path,next.body),next);
    assert(!denied.reached_backend && denied.wire.status==503);
    assert(store.record.last_ingress_nonce==3 && ingress.last_ingress_nonce()==3);
  }
  assert(!gateway.has_active_mission() && !gateway.emits_hardware_commands());
}
