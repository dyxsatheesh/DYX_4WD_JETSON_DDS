#include "backend_android_controls_fixture.hpp"
int main() {
  NativeOfflineAndroidMissionControls screen;
  auto ctx=android_controls_fixture();
  ctx.live.rtk_fixed=false;
  ctx.live.snapshot_fresh=false;
  ctx.ui_control_lease_visible=false;
  ctx.mission.safe_stop_intent=true;
  screen.show(ctx);
  assert(!screen.propose(AndroidMissionIntent::start,100.1).prepared);
  auto stop=screen.propose(AndroidMissionIntent::stop,100.2);
  assert(stop.prepared && stop.path==kBackendMissionStopPath);
  assert(stop.code=="ANDROID_STOP_PROPOSAL_UNCONFIRMED");
  android_no_actuation(stop);
  assert(screen.propose(AndroidMissionIntent::stop,100.3).sequence==stop.sequence+1);
  screen.disconnected();
  assert(!screen.request_pending());
  assert(screen.propose(AndroidMissionIntent::stop,100.4).code=="ANDROID_STOP_DELIVERY_UNAVAILABLE");
  auto c2=android_controls_fixture(MissionStage::await_next);
  c2.operator_confirmed_skip=false;
  screen.show(c2);
  assert(screen.propose(AndroidMissionIntent::skip,100.5).code=="ANDROID_SKIP_CONFIRMATION_REQUIRED");
  c2.operator_confirmed_skip=true;
  c2.unsaved_mission_edits=true;
  screen.show(c2);
  assert(screen.propose(AndroidMissionIntent::skip,100.6).code=="ANDROID_UNSAVED_MISSION_EDITS");
  c2.unsaved_mission_edits=false;
  screen.show(c2);
  auto skip=screen.propose(AndroidMissionIntent::skip,100.7);
  assert(skip.prepared && skip.path==kBackendMissionSkipPath);
  android_no_actuation(skip);
  auto malicious=android_controls_accepted();malicious.hardware_command_emitted=true;
  assert(screen.observe_backend(skip.sequence,malicious,100.8).code=="ANDROID_UNSAFE_REPLY_REJECTED");
}
