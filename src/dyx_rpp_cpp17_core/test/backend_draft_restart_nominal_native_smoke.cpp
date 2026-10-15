#include "backend_draft_restart_fence_fixture.hpp"
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
  auto screen=sync_screen();
  app.show(screen);
  assert(restart_sync(app,boundary,source,10.0,1).editor_resynchronized);
  assert(source.px4_calls==0);
  assert(restart_mutate(app,boundary,source,AndroidEditorIntent::upload,10.1,2)
         .requires_authoritative_resync);
  assert(restart_sync(app,boundary,source,10.2,3).draft.revision==1);
  screen.marking_points[1].x=5.2;
  app.show(screen);
  assert(restart_mutate(app,boundary,source,AndroidEditorIntent::update,10.3,4)
         .verified_backend_decision);
  assert(restart_sync(app,boundary,source,10.4,5).draft.revision==2);
  screen.unsaved_local_changes=false;
  screen.ui_confirmed_load=true;
  app.show(screen);
  assert(restart_mutate(app,boundary,source,AndroidEditorIntent::load,10.5,6)
         .loaded_ready_not_started);
  assert(restart_sync(app,boundary,source,10.6,7).draft.loaded);
  assert(store.commits==7 && store.record.last_ingress_nonce==7 &&
         store.record.generation==8 && ingress.last_ingress_nonce()==7);
  assert(source.px4_calls==3);
  assert(gateway.has_active_mission() &&
         gateway.mission().stage()==MissionStage::ready);
  assert(!gateway.emits_hardware_commands());
}
