#include "backend_draft_ingress_lab_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  NativeOfflineDraftIngressLab ingress(wire);
  NativeOfflineAndroidDraftCoordinator app;
  auto screen=sync_screen();
  app.show(screen);
  auto empty=lab_sync(app,ingress,10.0,1);
  assert(empty.editor_resynchronized && !empty.draft_present);
  auto upload=lab_mutate(app,ingress,AndroidEditorIntent::upload,10.1,2);
  assert(upload.verified_backend_decision && upload.requires_authoritative_resync);
  assert(!gateway.has_active_mission() && !gateway.emits_hardware_commands());
  auto preview=lab_sync(app,ingress,10.2,3);
  assert(preview.editor_resynchronized && preview.draft.revision==1);
  screen.marking_points[1].x=4.1;
  app.show(screen);
  auto update=lab_mutate(app,ingress,AndroidEditorIntent::update,10.3,4);
  assert(update.verified_backend_decision);
  auto revised=lab_sync(app,ingress,10.4,5);
  assert(revised.editor_resynchronized && revised.draft.revision==2);
  screen.unsaved_local_changes=false;
  screen.ui_confirmed_load=true;
  app.show(screen);
  auto load=lab_mutate(app,ingress,AndroidEditorIntent::load,10.5,6);
  assert(load.verified_backend_decision && load.loaded_ready_not_started);
  auto loaded=lab_sync(app,ingress,10.6,7);
  assert(loaded.editor_resynchronized && loaded.draft.loaded);
  assert(gateway.has_active_mission());
  assert(gateway.mission().stage()==MissionStage::ready);
  assert(!gateway.emits_hardware_commands());
  assert(ingress.last_ingress_nonce()==7 && ingress.session_bound());
}
