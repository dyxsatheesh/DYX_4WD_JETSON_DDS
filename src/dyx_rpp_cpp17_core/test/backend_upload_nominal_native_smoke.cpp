#include "backend_upload_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload upload(gateway);
  assert(upload.handle(draft_req()).http_status == 404);
  auto input=upload_req(kBackendMissionUploadPath, 1, 10);
  auto staged=upload.handle(input);
  assert(staged.accepted() && staged.code=="UPLOAD_PREVIEW_READY");
  assert(staged.revision == 1 && valid_path_signature(staged.signature));
  assert(staged.marking_count==2 && staged.navigation_count==41);
  assert(staged.dummy_count==0 && staged.preview_only && !staged.loaded);
  assert(!gateway.has_active_mission());
  const auto view=upload.handle(draft_req());
  assert(view.accepted() && view.signature==staged.signature);
  assert(upload.preview_snapshot() && upload.preview_snapshot()->signature==staged.signature);
  assert(upload.preview_snapshot()->marking_points[1].x == 2);
  assert(!view.hardware_command_emitted);
  auto commit=load_req(staged,2,11);
  const auto loaded=upload.handle(commit);
  assert(loaded.accepted() && loaded.code=="MISSION_LOADED_READY_NOT_STARTED");
  assert(loaded.loaded && !loaded.preview_only && upload.loaded());
  assert(gateway.has_active_mission());
  assert(gateway.mission().stage()==MissionStage::ready);
  assert(gateway.mission().records().size()==2);
  assert(gateway.mission().records().at(0).goal.terminal_identity.find("RUN:RUN_H4E_0001")!=std::string::npos);
  assert(!loaded.hardware_command_emitted && !gateway.emits_hardware_commands());
  assert(upload.handle(commit).http_status==409);
  assert(upload.handle(update_req(staged,3,12)).http_status==409);
  assert(!upload.discard_unloaded(true,true));
}
