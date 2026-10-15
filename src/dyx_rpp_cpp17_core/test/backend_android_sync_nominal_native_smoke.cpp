#include "backend_android_sync_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire server(backend);
  NativeOfflineAndroidMissionEditor editor;
  NativeOfflineAndroidDraftSync sync;
  NativeOfflineAndroidDraftHttpCodec codec;
  auto screen=sync_screen();
  editor.show(screen);

  // Authenticated GET 404 is the only condition permitting initial upload.
  auto empty=sync_execute(sync,editor,server,screen.mission_run_id,10.0,10.05);
  assert(empty.verified && empty.editor_resynchronized && !empty.draft_present);
  assert(empty.code=="ANDROID_DRAFT_CONFIRMED_ABSENT");
  sync_no_hardware(empty);

  auto upload_proposal=editor.propose(AndroidEditorIntent::upload,10.10);
  auto upload=codec.encode(upload_proposal);
  assert(upload.valid);
  auto upload_wire=server.handle(sync_trusted(upload.method,upload.path,upload.body,10.11));
  auto uploaded=codec.decode(codec_verified_response(upload_wire,upload,10.10,10.12),upload);
  assert(uploaded.valid);
  auto upload_ack=editor.observe_backend(upload.sequence,uploaded.backend,10.13);
  assert(upload_ack.backend_decision_accepted && editor.requires_resync());
  auto first=sync_execute(sync,editor,server,screen.mission_run_id,10.20,10.24);
  assert(first.verified && first.editor_resynchronized && first.draft_present);
  assert(first.draft.revision==1 && first.draft.preview_only && !first.draft.loaded);
  assert(!editor.requires_resync());

  screen.backend_draft_present=true;
  screen.backend_draft=first.draft;
  screen.backend_draft_received_sec=10.24;
  screen.marking_points[1].x=3.5;
  screen.unsaved_local_changes=true;
  editor.show(screen);
  auto update=codec.encode(editor.propose(AndroidEditorIntent::update,10.30));
  assert(update.valid);
  auto updated_wire=server.handle(sync_trusted(update.method,update.path,update.body,10.31));
  auto updated=codec.decode(codec_verified_response(updated_wire,update,10.30,10.32),update);
  assert(updated.valid && updated.backend.revision==2);
  assert(editor.observe_backend(update.sequence,updated.backend,10.33).backend_decision_accepted);
  auto second=sync_execute(sync,editor,server,screen.mission_run_id,10.40,10.44);
  assert(second.editor_resynchronized && second.draft.revision==2);
  assert(second.draft.signature!=first.draft.signature);

  screen.backend_draft_present=true;
  screen.backend_draft=second.draft;
  screen.backend_draft_received_sec=10.44;
  screen.unsaved_local_changes=false;
  screen.ui_confirmed_load=true;
  editor.show(screen);
  auto load=codec.encode(editor.propose(AndroidEditorIntent::load,10.50));
  assert(load.valid && load.body.find("marking_points")==std::string::npos);
  auto loaded_wire=server.handle(sync_trusted(load.method,load.path,load.body,10.51));
  auto loaded=codec.decode(codec_verified_response(loaded_wire,load,10.50,10.52),load);
  assert(loaded.valid && loaded.backend.loaded);
  assert(editor.observe_backend(load.sequence,loaded.backend,10.53).backend_decision_accepted);
  auto third=sync_execute(sync,editor,server,screen.mission_run_id,10.60,10.64);
  assert(third.editor_resynchronized && third.draft.loaded && !third.draft.preview_only);
  assert(third.code=="ANDROID_DRAFT_LOADED_SYNCED_NOT_STARTED");
  sync_no_hardware(third);
  assert(gateway.has_active_mission() && gateway.mission().stage()==MissionStage::ready);
  assert(!gateway.emits_hardware_commands());
}
