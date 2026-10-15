#include "backend_android_coordinator_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire server(backend);
  NativeOfflineAndroidDraftCoordinator app;
  auto screen=sync_screen();
  app.show(screen);
  assert(coordinate_sync(app,server,10.0).editor_resynchronized);
  auto first=app.begin_mutation(AndroidEditorIntent::upload,10.1);
  assert(first.valid);
  const auto accepted=server.handle(codec_trusted_http(first,10.11));
  screen.mission_run_id="RUN_H4S_NEW";
  app.show(screen); // switch mission must fence the old request
  assert(!app.mutation_pending() && app.requires_resync());
  auto old=app.complete_mutation(first,
      codec_verified_response(accepted,first,10.1,10.13),10.14);
  assert(!old.verified_backend_decision);
  assert(!app.begin_mutation(AndroidEditorIntent::upload,10.15).valid);

  // A forged old GET response cannot establish state for the new mission.
  auto newer=app.begin_resync(10.2);
  assert(newer.valid);
  auto other=server.handle(sync_trusted("GET",newer.path,"",10.21));
  assert(other.status==403); // backend draft belongs to old mission
  auto denied=app.complete_resync(newer,sync_verified(newer,other,10.2,10.23),10.24);
  assert(!denied.editor_resynchronized && app.requires_resync());
  screen.ui_signed_in=false;
  app.show(screen);
  assert(!app.begin_resync(10.3).valid);
  assert(!app.begin_mutation(AndroidEditorIntent::upload,10.31).valid);
  assert(!gateway.has_active_mission() && !gateway.emits_hardware_commands());
}
