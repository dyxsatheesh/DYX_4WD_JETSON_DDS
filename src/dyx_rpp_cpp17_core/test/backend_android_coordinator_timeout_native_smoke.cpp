#include "backend_android_coordinator_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire server(backend);
  NativeOfflineAndroidDraftCoordinator app;
  auto screen=sync_screen();
  app.show(screen);
  assert(coordinate_sync(app,server,10.0).editor_resynchronized);
  auto upload=app.begin_mutation(AndroidEditorIntent::upload,10.1);
  assert(upload.valid);
  // Backend accepted POST; Android response got lost. Must not retry.
  auto response=server.handle(codec_trusted_http(upload,10.12));
  assert(response.status==200);
  assert(!app.expire_mutation(10.4).verified_backend_decision);
  auto expiration=app.expire_mutation(12.2);
  assert(expiration.code=="ANDROID_COORDINATOR_RESULT_UNKNOWN_RESYNC");
  assert(!app.mutation_pending() && app.requires_resync());
  assert(!app.complete_mutation(upload,
       codec_verified_response(response,upload,10.1,10.15),12.21)
       .verified_backend_decision);
  assert(!app.begin_mutation(AndroidEditorIntent::upload,12.22).valid);
  auto state=coordinate_sync(app,server,12.3);
  assert(state.editor_resynchronized && state.draft_present && state.draft.revision==1);
  assert(!app.requires_resync());
  assert(app.last_mutation_sequence()==1);
  assert(!gateway.has_active_mission() && !gateway.emits_hardware_commands());
}
