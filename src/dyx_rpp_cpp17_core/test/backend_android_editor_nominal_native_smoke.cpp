#include "backend_android_editor_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeOfflineAndroidMissionEditor editor;
  auto s=editor_screen();
  editor.show(s);
  assert(editor.synced_empty_draft(true,100));
  const auto upload=editor.propose(AndroidEditorIntent::upload,100.1);
  assert(upload.prepared && upload.sequence==1 &&
         upload.path==kBackendMissionUploadPath && upload.method=="POST");
  assert(upload.marking_points.size()==3 && upload.expected_revision==0);
  editor_non_actuating(upload);
  BackendMissionUploadRequest req;
  req.path=upload.path;
  req.mission_run_id=upload.mission_run_id;
  req.sequence=upload.sequence;
  req.marking_points=upload.marking_points;
  req.extension_enabled=upload.extension_enabled;
  req.extension_distance_m=upload.extension_distance_m;
  req.mission_mode=upload.mission_mode;
  // Backend adapter supplies these, NEVER Android request body:
  req.principal={true,true,true,"TRUSTED_OPERATOR_H4O"};
  req.trusted_px4_disarmed=true;
  req.host_received_sec=100.1;
  const auto first=backend.handle(req);
  assert(first.code=="UPLOAD_PREVIEW_READY");
  const auto ack1=editor.observe_backend(upload.sequence,first,100.2);
  assert(ack1.backend_decision_accepted && ack1.ready_to_load &&
         !ack1.backend_reports_loaded && editor.requires_resync());
  editor_non_actuating(ack1);
  auto draft_request=req;
  draft_request.path=kBackendMissionDraftPath;
  draft_request.method="GET";
  draft_request.principal.owns_control_lease=false;
  const auto draft1=backend.handle(draft_request);
  assert(editor.synced_from_backend(draft1,100.3,100.4));
  s.backend_draft_present=true;
  s.backend_draft=draft1;
  s.backend_draft_received_sec=100.3;
  s.marking_points={{0,0},{0,2},{3,2}};
  s.unsaved_local_changes=true;
  editor.show(s);
  const auto edit=editor.propose(AndroidEditorIntent::update,100.5);
  assert(edit.prepared && edit.path==kBackendMissionUpdatePath &&
         edit.sequence==2 && edit.expected_revision==1 &&
         edit.expected_signature==draft1.signature);
  req.path=edit.path;
  req.sequence=edit.sequence;
  req.host_received_sec=100.5;
  req.expected_revision=edit.expected_revision;
  req.expected_signature=edit.expected_signature;
  req.marking_points=edit.marking_points;
  const auto modified=backend.handle(req);
  assert(modified.code=="UPDATED_PREVIEW_READY" && modified.revision==2);
  const auto ack2=editor.observe_backend(edit.sequence,modified,100.6);
  assert(ack2.backend_decision_accepted && !ack2.backend_reports_loaded);
  assert(editor.synced_from_backend(backend.handle(draft_request),100.7,100.8));
  s.backend_draft=modified;
  s.backend_draft_received_sec=100.7;
  s.unsaved_local_changes=false;
  s.ui_confirmed_load=true;
  editor.show(s);
  const auto load=editor.propose(AndroidEditorIntent::load,100.9);
  assert(load.prepared && load.path==kBackendMissionLoadPath &&
         load.marking_points.empty() && load.sequence==3 &&
         load.expected_revision==2 && load.expected_signature==modified.signature);
  editor_non_actuating(load);
  req.path=load.path;
  req.sequence=load.sequence;
  req.host_received_sec=100.9;
  req.expected_revision=load.expected_revision;
  req.expected_signature=load.expected_signature;
  req.marking_points=load.marking_points;
  const auto loaded=backend.handle(req);
  assert(loaded.code=="MISSION_LOADED_READY_NOT_STARTED");
  const auto ack3=editor.observe_backend(load.sequence,loaded,101.0);
  assert(ack3.backend_decision_accepted && ack3.backend_reports_loaded &&
         !ack3.ready_to_load && !ack3.hardware_command_emitted);
  assert(gateway.mission().stage()==MissionStage::ready);
  assert(!gateway.emits_hardware_commands());
}
