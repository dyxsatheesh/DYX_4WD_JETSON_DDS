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
  auto screen=sync_screen();
  app.show(screen);
  assert(boundary_sync(app,boundary,source,10.0,1).editor_resynchronized);
  assert(source.px4_calls==0); // GET does not require PX4 evidence.
  assert(boundary_mutate(app,boundary,source,AndroidEditorIntent::upload,10.1,2)
         .requires_authoritative_resync);
  assert(boundary_sync(app,boundary,source,10.2,3).draft.revision==1);
  screen.marking_points[1].x=5.2;
  app.show(screen);
  assert(boundary_mutate(app,boundary,source,AndroidEditorIntent::update,10.3,4)
         .verified_backend_decision);
  assert(boundary_sync(app,boundary,source,10.4,5).draft.revision==2);
  screen.unsaved_local_changes=false;
  screen.ui_confirmed_load=true;
  app.show(screen);
  assert(boundary_mutate(app,boundary,source,AndroidEditorIntent::load,10.5,6)
         .loaded_ready_not_started);
  assert(boundary_sync(app,boundary,source,10.6,7).draft.loaded);
  assert(source.px4_calls==3 && ingress.last_ingress_nonce()==7);
  assert(gateway.has_active_mission() &&
         gateway.mission().stage()==MissionStage::ready);
  assert(!gateway.emits_hardware_commands());
}
