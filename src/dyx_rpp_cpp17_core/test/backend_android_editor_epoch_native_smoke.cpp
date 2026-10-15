#include "backend_android_editor_fixture.hpp"
int main() {
  NativeOfflineAndroidMissionEditor editor;
  auto s=editor_screen();
  editor.show(s);
  assert(editor.synced_empty_draft(true,100));
  const auto first=editor.propose(AndroidEditorIntent::upload,100.1);
  assert(first.prepared);
  s.mission_run_id="RUN_H4O_0002";
  editor.show(s);
  assert(!editor.pending() && editor.requires_resync());
  assert(editor.observe_backend(first.sequence,
         editor_response("UPLOAD_PREVIEW_READY",1),100.2).code==
         "ANDROID_EDITOR_REPLY_UNMATCHED_OR_REPLAYED");
  assert(editor.synced_empty_draft(true,100.2));
  const auto second=editor.propose(AndroidEditorIntent::upload,100.3);
  assert(second.prepared && second.sequence>first.sequence &&
         second.mission_run_id=="RUN_H4O_0002");
  editor.disconnected();
  assert(!editor.pending() && editor.requires_resync());
  assert(editor.observe_backend(second.sequence,
         editor_response("UPLOAD_PREVIEW_READY",1),100.4).code==
         "ANDROID_EDITOR_REPLY_UNMATCHED_OR_REPLAYED");
  s.ui_network_connected=true;
  editor.show(s);
  assert(editor.propose(AndroidEditorIntent::upload,100.5).code==
         "ANDROID_EDITOR_BACKEND_RESYNC_REQUIRED");
  assert(editor.synced_empty_draft(true,100.5));
  s.backend_draft_present=true;
  s.backend_draft=editor_backend_draft();
  s.backend_draft.signature="MALFORMED";
  s.backend_draft_received_sec=100.5;
  editor.show(s);
  assert(editor.propose(AndroidEditorIntent::update,100.5).code==
         "ANDROID_EDITOR_FRESH_PREVIEW_REQUIRED");
  assert(!editor.synced_from_backend(s.backend_draft,100.6,100.7));
  s.backend_draft=editor_backend_draft();
  editor.show(s);
  assert(editor.synced_from_backend(s.backend_draft,100.8,100.9));
  const auto upd=editor.propose(AndroidEditorIntent::update,101);
  assert(upd.prepared && upd.sequence>second.sequence);
  s.ui_signed_in=false;
  editor.show(s);
  assert(!editor.pending());
  assert(editor.propose(AndroidEditorIntent::update,101.1).code==
         "ANDROID_EDITOR_SIGN_IN_REQUIRED");
}
