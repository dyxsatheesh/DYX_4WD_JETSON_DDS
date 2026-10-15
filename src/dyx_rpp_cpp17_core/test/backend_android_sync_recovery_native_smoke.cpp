#include "backend_android_sync_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire server(backend);
  NativeOfflineAndroidMissionEditor editor;
  auto screen=sync_screen();
  editor.show(screen);
  NativeOfflineAndroidDraftSync sync;
  auto missing=sync_execute(sync,editor,server,screen.mission_run_id,10.0,10.02);
  assert(missing.editor_resynchronized);
  auto prepared=editor.propose(AndroidEditorIntent::upload,10.1);
  NativeOfflineAndroidDraftHttpCodec codec;
  auto encoded=codec.encode(prepared);
  assert(encoded.valid);
  // Backend accepts upload but the Android reply is LOST. Never repeat POST.
  assert(server.handle(sync_trusted("POST",encoded.path,encoded.body,10.11)).status==200);
  auto timeout=editor.expire(12.2);
  assert(timeout.code=="ANDROID_EDITOR_TIMEOUT_UNKNOWN_RESYNC");
  assert(editor.requires_resync());
  auto after=sync_execute(sync,editor,server,screen.mission_run_id,12.3,12.34);
  assert(after.verified && after.editor_resynchronized && after.draft_present);
  assert(after.draft.revision==1 && backend.has_draft());
  assert(gateway.has_active_mission()==false);

  // Independently verified transport cannot apply data to a new mission UI.
  auto request=sync.begin(screen.mission_run_id,12.4);
  auto wire=server.handle(sync_trusted("GET",request.path,"",12.42));
  editor.show(sync_screen("RUN_H4R_0002"));
  auto denied=sync.complete(request,sync_verified(request,wire,12.4,12.45),12.46,editor);
  assert(!denied.verified && !denied.editor_resynchronized);
  assert(editor.requires_resync());
  assert(!gateway.emits_hardware_commands());

  // Exact foreign-owner 403 MUST NOT be treated as a missing draft.
  auto other=sync.begin("RUN_H4R_0002",12.5);
  auto forbidden=server.handle(sync_trusted("GET",other.path,"",12.52));
  assert(forbidden.status==403);
  denied=sync.complete(other,sync_verified(other,forbidden,12.5,12.55),12.56,editor);
  assert(!denied.verified && !denied.editor_resynchronized);
}
