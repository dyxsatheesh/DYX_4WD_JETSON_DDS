#include "backend_draft_http_framing_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  NativeOfflineDraftIngressLab ingress(wire);
  NativeOfflineDraftHttpFramingLab framing(ingress);
  NativeOfflineAndroidDraftCoordinator app;
  auto screen=sync_screen();
  app.show(screen);
  assert(framed_sync(app,framing,10.0,1).editor_resynchronized);
  assert(framed_mutation(app,framing,AndroidEditorIntent::upload,10.1,2)
      .requires_authoritative_resync);
  assert(framed_sync(app,framing,10.2,3).draft.revision==1);
  screen.marking_points[1].x=5.2;
  app.show(screen);
  assert(framed_mutation(app,framing,AndroidEditorIntent::update,10.3,4)
      .verified_backend_decision);
  assert(framed_sync(app,framing,10.4,5).draft.revision==2);
  screen.unsaved_local_changes=false;
  screen.ui_confirmed_load=true;
  app.show(screen);
  assert(framed_mutation(app,framing,AndroidEditorIntent::load,10.5,6)
      .loaded_ready_not_started);
  assert(framed_sync(app,framing,10.6,7).draft.loaded);
  assert(gateway.has_active_mission() &&
      gateway.mission().stage()==MissionStage::ready);
  assert(!gateway.emits_hardware_commands());
  assert(ingress.last_ingress_nonce()==7);
}
