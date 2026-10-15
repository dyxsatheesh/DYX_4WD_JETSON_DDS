#include "backend_android_coordinator_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire server(backend);
  NativeOfflineAndroidDraftCoordinator app;
  auto screen=sync_screen();
  app.show(screen);
  assert(app.requires_resync());
  auto no_draft=coordinate_sync(app,server,10.0);
  assert(no_draft.verified && no_draft.editor_resynchronized && !no_draft.draft_present);
  assert(!app.requires_resync());

  auto upload=coordinate_mutate(app,server,AndroidEditorIntent::upload,10.1);
  assert(upload.verified_backend_decision && upload.requires_authoritative_resync);
  coordinate_safe(upload);
  assert(app.requires_resync() && !gateway.has_active_mission());
  auto preview=coordinate_sync(app,server,10.2);
  assert(preview.editor_resynchronized && preview.draft.revision==1);
  assert(!app.requires_resync());

  screen.marking_points[1].x=3.25;
  app.show(screen); // repeat UI screen cannot replace authenticated revision
  auto update=coordinate_mutate(app,server,AndroidEditorIntent::update,10.3);
  assert(update.verified_backend_decision);
  auto updated=coordinate_sync(app,server,10.4);
  assert(updated.editor_resynchronized && updated.draft.revision==2);

  screen.unsaved_local_changes=false;
  screen.ui_confirmed_load=true;
  app.show(screen);
  auto loaded=coordinate_mutate(app,server,AndroidEditorIntent::load,10.5);
  assert(loaded.verified_backend_decision && loaded.loaded_ready_not_started);
  coordinate_safe(loaded);
  assert(app.requires_resync());
  auto current=coordinate_sync(app,server,10.6);
  assert(current.verified && current.editor_resynchronized && current.draft.loaded);
  assert(gateway.has_active_mission() && gateway.mission().stage()==MissionStage::ready);
  assert(!gateway.emits_hardware_commands());
  assert(!app.begin_mutation(AndroidEditorIntent::load,10.7).valid);
}
