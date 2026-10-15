#include "backend_android_editor_fixture.hpp"
int main() {
  NativeOfflineAndroidMissionEditor editor;
  auto s=editor_screen();
  editor.show(s);
  assert(editor.propose(AndroidEditorIntent::upload,100).code==
         "ANDROID_EDITOR_BACKEND_RESYNC_REQUIRED");
  assert(editor.synced_empty_draft(true,100));
  s.ui_px4_disarmed_indicator=false;
  editor.show(s);
  assert(editor.propose(AndroidEditorIntent::upload,101).code==
         "ANDROID_EDITOR_UI_DISARM_REQUIRED");
  s.ui_px4_disarmed_indicator=true;
  s.ui_control_lease_visible=false;
  editor.show(s);
  assert(editor.propose(AndroidEditorIntent::upload,101).code==
         "ANDROID_EDITOR_BACKEND_RESYNC_REQUIRED");
  assert(editor.synced_empty_draft(true,101));
  assert(editor.propose(AndroidEditorIntent::upload,101).code==
         "ANDROID_EDITOR_UI_LEASE_REQUIRED");
  s.ui_control_lease_visible=true;
  s.marking_points={{0,0},{0,0}};
  editor.show(s);
  assert(editor.synced_empty_draft(true,101));
  assert(editor.propose(AndroidEditorIntent::upload,101).code==
         "ANDROID_EDITOR_INVALID_LOCAL_GEOMETRY");
  s.marking_points={{0,0},{2,0},{2,3}};
  s.extension_distance_m=std::numeric_limits<double>::infinity();
  editor.show(s);
  assert(editor.propose(AndroidEditorIntent::upload,101).code==
         "ANDROID_EDITOR_INVALID_LOCAL_GEOMETRY");
  s.extension_distance_m=4.0;
  editor.show(s);
  const auto p=editor.propose(AndroidEditorIntent::upload,101);
  assert(p.prepared);
  assert(editor.propose(AndroidEditorIntent::upload,101.1).code==
         "ANDROID_EDITOR_REQUEST_PENDING");
  auto dishonest=editor_response("UPLOAD_PREVIEW_READY",1);
  dishonest.hardware_command_emitted=true;
  const auto a=editor.observe_backend(p.sequence,dishonest,101.2);
  assert(!a.backend_decision_accepted && editor.requires_resync());
  editor_non_actuating(a);
  assert(editor.propose(AndroidEditorIntent::upload,101.3).code==
         "ANDROID_EDITOR_BACKEND_RESYNC_REQUIRED");
  assert(editor.synced_from_backend(editor_backend_draft(),101.4,101.5));
  s.backend_draft_present=true;s.backend_draft=editor_backend_draft();
  s.backend_draft_received_sec=101.4;
  s.unsaved_local_changes=true;
  editor.show(s);
  assert(editor.propose(AndroidEditorIntent::load,101.5).code==
         "ANDROID_EDITOR_UPDATE_BEFORE_LOAD");
  s.unsaved_local_changes=false;
  editor.show(s);
  assert(editor.propose(AndroidEditorIntent::load,101.5).code==
         "ANDROID_EDITOR_LOAD_CONFIRMATION_REQUIRED");
  s.ui_confirmed_load=true;
  s.marking_points.pop_back();
  editor.show(s);
  assert(editor.propose(AndroidEditorIntent::load,101.5).code==
         "ANDROID_EDITOR_PREVIEW_COUNT_CONFLICT");
  s.marking_points.push_back({2,3});
  editor.show(s);
  const auto ld=editor.propose(AndroidEditorIntent::load,101.6);
  assert(ld.prepared && ld.marking_points.empty());
  editor_non_actuating(ld);
}
