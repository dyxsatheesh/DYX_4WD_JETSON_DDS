#include "backend_android_coordinator_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire server(backend);
  NativeOfflineAndroidDraftCoordinator app;
  auto screen=sync_screen();
  screen.backend_draft_present=true; // forged UI cache is not trusted
  screen.backend_draft.revision=999;
  screen.backend_draft.signature="AAAAAAAAAAAAAAAA";
  screen.unsaved_local_changes=false;
  screen.ui_confirmed_load=true;
  app.show(screen);
  assert(!app.begin_mutation(AndroidEditorIntent::load,10.0).valid);

  auto get=app.begin_resync(10.1);
  auto wire=server.handle(sync_trusted("GET",get.path,"",10.11));
  auto untrusted=sync_verified(get,wire,10.1,10.13);
  untrusted.verified_authenticated_transport=false;
  auto denial=app.complete_resync(get,untrusted,10.14);
  assert(!denial.editor_resynchronized && app.requires_resync());
  assert(coordinate_sync(app,server,10.2).editor_resynchronized);

  screen.unsaved_local_changes=true;
  app.show(screen);
  auto request=app.begin_mutation(AndroidEditorIntent::upload,10.3);
  assert(request.valid);
  assert(!app.begin_resync(10.31).valid);
  auto server_wire=server.handle(codec_trusted_http(request,10.32));
  auto trusted=codec_verified_response(server_wire,request,10.3,10.34);
  // Same sequence/path but tampered original BODY must not be accepted.
  auto tampered=request;
  tampered.body+=" ";
  auto bad=app.complete_mutation(tampered,trusted,10.35);
  assert(!bad.verified_backend_decision && app.requires_resync());
  coordinate_safe(bad);
  assert(!app.complete_mutation(request,trusted,10.36).verified_backend_decision);
  assert(!app.begin_mutation(AndroidEditorIntent::upload,10.37).valid);
  assert(coordinate_sync(app,server,10.4).draft.revision==1);
  assert(!gateway.has_active_mission() && !gateway.emits_hardware_commands());
}
