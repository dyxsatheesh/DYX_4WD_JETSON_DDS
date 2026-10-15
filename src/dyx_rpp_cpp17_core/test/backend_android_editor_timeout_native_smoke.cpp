#include "backend_android_editor_fixture.hpp"
int main() {
  NativeOfflineAndroidMissionEditor editor;
  auto s=editor_screen();
  editor.show(s);
  assert(editor.synced_empty_draft(true,50));
  const auto first=editor.propose(AndroidEditorIntent::upload,51);
  assert(first.prepared);
  assert(editor.expire(52).code=="ANDROID_EDITOR_WAITING_FOR_BACKEND");
  const auto late=editor.expire(54);
  assert(late.code=="ANDROID_EDITOR_TIMEOUT_UNKNOWN_RESYNC");
  assert(!editor.pending() && editor.requires_resync());
  assert(editor.observe_backend(first.sequence,
          editor_response("UPLOAD_PREVIEW_READY",1),54.1).code==
         "ANDROID_EDITOR_REPLY_UNMATCHED_OR_REPLAYED");
  assert(editor.propose(AndroidEditorIntent::upload,54.2).code==
         "ANDROID_EDITOR_BACKEND_RESYNC_REQUIRED");
  assert(editor.synced_empty_draft(true,54.3));
  const auto second=editor.propose(AndroidEditorIntent::upload,54.4);
  assert(second.prepared && second.sequence>first.sequence);
  const auto malformed=editor_response("UPLOAD_PREVIEW_READY",2);
  const auto a=editor.observe_backend(second.sequence,malformed,54.5);
  assert(!a.backend_decision_accepted && editor.requires_resync());
  assert(editor.expire(52).code=="ANDROID_EDITOR_CLOCK_FAULT_LATCHED");
  assert(editor.clock_fault_latched());
  assert(!editor.recover_disarmed(false,true,60));
  assert(!editor.recover_disarmed(true,false,60));
  assert(editor.recover_disarmed(true,true,60));
  editor.show(s);
  assert(editor.synced_empty_draft(true,60.1));
  const auto third=editor.propose(AndroidEditorIntent::upload,60.2);
  assert(third.prepared && third.sequence>second.sequence);
}
