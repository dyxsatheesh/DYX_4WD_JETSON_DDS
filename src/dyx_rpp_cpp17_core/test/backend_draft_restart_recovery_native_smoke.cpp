#include "backend_draft_restart_fence_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  RestartLabMemoryStore store;
  NativeOfflineAndroidDraftCoordinator app;
  app.show(sync_screen());
  AndroidDraftEncodedRequest upload;
  std::string raw;
  {
    NativeOfflineDraftIngressLab ingress(wire);
    NativeOfflineDraftHttpFramingLab framing(ingress);
    BoundaryLabFixtureSource source;
    NativeOfflineDraftRestartFenceSource fence(source,store);
    NativeOfflineDraftSecurityBoundaryLab boundary(framing,fence);
    assert(restart_sync(app,boundary,source,40.0,1).editor_resynchronized);
    upload=app.begin_mutation(AndroidEditorIntent::upload,40.1);
    assert(upload.valid);
    raw=framed_lab_request(upload.method,upload.path,upload.body);
    source.set_time_nonce(40.12,2);
    auto accepted=boundary.mutation(raw,upload);
    assert(accepted.reached_backend && accepted.wire.status==200);
    assert(backend.has_draft() && store.record.last_ingress_nonce==2);
    // Deliberately lose the HTTP response; the Android editor knows nothing.
  }
  assert(!app.expire_mutation(42.2).verified_backend_decision);
  assert(app.requires_resync());
  {
    // Recreate process-local ingress/boundary but preserve durable mock store.
    NativeOfflineDraftIngressLab ingress(wire);
    NativeOfflineDraftHttpFramingLab framing(ingress);
    BoundaryLabFixtureSource source;
    NativeOfflineDraftRestartFenceSource fence(source,store);
    NativeOfflineDraftSecurityBoundaryLab boundary(framing,fence);
    source.set_time_nonce(42.22,2);
    auto denied=boundary.mutation(raw,upload);
    assert(!denied.reached_backend && denied.wire.status==503);
    assert(ingress.last_ingress_nonce()==0);
    source.set_time_nonce(42.24,3);
    auto repeated=boundary.mutation(raw,upload);
    assert(repeated.reached_backend && repeated.wire.status==409);
    assert(!app.begin_mutation(AndroidEditorIntent::load,42.26).valid);
    assert(restart_sync(app,boundary,source,42.3,4).draft.revision==1);
    assert(!app.requires_resync() && store.record.last_ingress_nonce==4);
  }
  assert(!gateway.has_active_mission() && !gateway.emits_hardware_commands());
}
