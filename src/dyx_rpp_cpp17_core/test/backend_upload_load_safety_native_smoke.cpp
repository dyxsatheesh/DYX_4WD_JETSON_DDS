#include "backend_upload_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload upload(gateway);
  const auto staged=upload.handle(upload_req(kBackendMissionUploadPath,1,10));
  assert(staged.accepted());
  auto denied=load_req(staged,2,11);
  denied.trusted_px4_disarmed=false;
  assert(upload.handle(denied).code=="PX4_DISARM_REQUIRED");
  assert(!gateway.has_active_mission());
  auto safe=load_req(staged,3,12);
  assert(upload.handle(safe).accepted());
  assert(gateway.has_active_mission());
  gateway.force_stop();
  assert(!gateway.has_active_mission() && gateway.safe_stop_requested());
  // Run identity is one-shot even after STOP. A different uploader cannot replay.
  NativeBackendMissionUpload second(gateway);
  auto other=upload_req(kBackendMissionUploadPath,1,20);
  assert(second.handle(other).accepted());
  assert(second.handle(load_req(second.handle(draft_req()),2,21)).code=="MISSION_RUN_ID_REUSED");
  assert(!gateway.has_active_mission());
  assert(!second.loaded());
  // A new run ID can only be staged after explicit authorised disarmed discard.
  assert(second.discard_unloaded(true,true));
  other.mission_run_id="RUN_H4E_0002";
  assert(second.handle(other).accepted());
  auto p=draft_req();p.mission_run_id="RUN_H4E_0002";
  auto ready=second.handle(p);
  assert(ready.accepted());
  auto next=load_req(ready,2,21);
  next.mission_run_id="RUN_H4E_0002";
  assert(second.handle(next).accepted());
  assert(gateway.mission().stage()==MissionStage::ready);
}
