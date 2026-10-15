#include "backend_draft_restart_fence_fixture.hpp"
#include <limits>
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  NativeOfflineDraftIngressLab ingress(wire);
  NativeOfflineDraftHttpFramingLab framing(ingress);
  BoundaryLabFixtureSource source;
  RestartLabMemoryStore store;
  NativeOfflineDraftRestartFenceSource fence(source,store);
  NativeOfflineDraftSecurityBoundaryLab boundary(framing,fence);
  NativeOfflineAndroidDraftCoordinator app;
  app.show(sync_screen());
  const auto get=app.begin_resync(30.0);
  assert(get.valid);
  const auto raw=framed_lab_request(get.method,get.path,get.body);
  auto attempt=[&](double now,std::uint64_t nonce) {
    source.set_time_nonce(now,nonce);
    return boundary.snapshot(raw,get);
  };
  store.present=false;
  assert(attempt(30.02,1).wire.status==503);
  store.present=true;
  store.available=false;
  assert(attempt(30.02,1).wire.status==503);
  store.available=true;
  store.record.generation=0; // Corrupted record never resets automatically.
  assert(attempt(30.02,1).wire.status==503);
  store.record.generation=1;
  store.write_available=false;
  assert(attempt(30.02,1).wire.status==503);
  store.write_available=true;
  store.force_conflict=true; // A lost CAS race must not reach backend.
  assert(attempt(30.02,1).wire.status==503);
  store.force_conflict=false;
  assert(store.commits==0 && store.record.last_ingress_nonce==0);
  const auto ok=attempt(30.02,1);
  assert(ok.reached_backend && ok.trusted_reply_provenance); // No draft yet.
  assert(store.commits==1 && store.record.last_ingress_nonce==1);
  assert(attempt(30.03,1).wire.status==503); // Consumed nonce.
  assert(attempt(29.0,2).wire.status==503); // Rollback across restart.
  store.record.generation=std::numeric_limits<std::uint64_t>::max();
  assert(attempt(30.04,2).wire.status==503); // Generation overflow.
  assert(ingress.last_ingress_nonce()==1 && store.commits==1);
  assert(!gateway.has_active_mission() && !gateway.emits_hardware_commands());
}
