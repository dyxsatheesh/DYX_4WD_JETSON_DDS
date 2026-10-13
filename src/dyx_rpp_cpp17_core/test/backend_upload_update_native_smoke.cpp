#include "backend_upload_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload upload(gateway);
  auto initial=upload.handle(upload_req(kBackendMissionUploadPath,1,1));
  assert(initial.accepted() && initial.navigation_count==41);
  auto edit=update_req(initial,2,2);
  edit.extension_enabled=true;
  edit.extension_distance_m=4.0;
  edit.marking_points={{0,0},{0,8},{1.5,8},{1.5,0}};
  // Stale LOAD must not commit prior path while operator edits locally.
  auto staged=upload.handle(edit);
  assert(staged.accepted() && staged.code=="UPDATED_PREVIEW_READY");
  assert(staged.revision==2 && staged.signature!=initial.signature);
  assert(staged.marking_count==4 && staged.dummy_count==1);
  assert(staged.navigation_count>initial.navigation_count);
  assert(!gateway.has_active_mission());
  assert(upload.handle(load_req(initial,3,3)).code=="DRAFT_REVISION_CONFLICT");
  // Rejection doesn't discard the updated preview.
  assert(upload.handle(draft_req()).signature==staged.signature);
  auto bad=update_req(staged,4,4);
  bad.marking_points={{0,0},{0,0}};
  assert(upload.handle(bad).http_status==422);
  assert(upload.handle(draft_req()).signature==staged.signature);
  auto wrong=load_req(staged,5,5);
  wrong.marking_points={{0,0},{1,1}};
  assert(upload.handle(wrong).code=="LOAD_MUST_USE_SAVED_PREVIEW");
  const auto done=upload.handle(load_req(staged,6,6));
  assert(done.accepted() && done.loaded && done.signature==staged.signature);
  assert(gateway.mission().records().size()==4);
  assert(gateway.mission().records().at(2).goal.goal_label=="P0003");
  assert(!done.hardware_command_emitted);
}
