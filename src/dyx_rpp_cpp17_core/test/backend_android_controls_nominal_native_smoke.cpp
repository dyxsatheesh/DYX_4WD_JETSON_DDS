#include "backend_android_controls_fixture.hpp"
int main() {
  NativeOfflineAndroidMissionControls screen;
  screen.show(android_controls_fixture());
  auto first=screen.propose(AndroidMissionIntent::start,100.1);
  assert(first.prepared && first.method=="POST" && first.path==kBackendMissionStartPath);
  assert(first.json_body=="{\"mission_run_id\":\"run_ph2h4n\",\"sequence\":1}");
  assert(first.sequence==1 && screen.request_pending());
  android_no_actuation(first);
  assert(screen.propose(AndroidMissionIntent::pause,100.11).code=="ANDROID_REQUEST_PENDING");
  auto accepted=screen.observe_backend(first.sequence,android_controls_accepted(),100.2);
  assert(accepted.backend_decision_reported);
  assert(accepted.code=="ANDROID_BACKEND_DECISION_ACCEPTED_NOT_ACTUATED");
  android_no_actuation(accepted);
  auto replay=screen.observe_backend(first.sequence,android_controls_accepted(),100.3);
  assert(replay.code=="ANDROID_REPLY_UNMATCHED_OR_REPLAYED");
  auto running=android_controls_fixture(MissionStage::approaching);
  screen.show(running);
  auto pause=screen.propose(AndroidMissionIntent::pause,100.4);
  assert(pause.prepared && pause.sequence==2 && pause.path==kBackendMissionPausePath);
  auto denied=android_controls_accepted();denied.http_status=409;denied.code="INVALID_MISSION_STAGE";
  assert(screen.observe_backend(pause.sequence,denied,100.5).code==
      "ANDROID_BACKEND_DECISION_DENIED_OR_UNKNOWN");
  screen.show(android_controls_fixture(MissionStage::paused));
  const auto resume=screen.propose(AndroidMissionIntent::resume,100.6);
  assert(resume.prepared && resume.path==kBackendMissionResumePath);
  android_no_actuation(resume);
}
