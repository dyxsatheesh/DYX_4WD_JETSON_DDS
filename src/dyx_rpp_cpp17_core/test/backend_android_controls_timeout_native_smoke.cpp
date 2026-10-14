#include "backend_android_controls_fixture.hpp"
int main() {
  NativeOfflineAndroidMissionControls screen;
  screen.show(android_controls_fixture());
  auto first=screen.propose(AndroidMissionIntent::start,100.1);
  assert(first.prepared);
  auto still=screen.expire(101.0);
  assert(still.code=="ANDROID_AWAITING_BACKEND_DECISION");
  auto timeout=screen.expire(102.11);
  assert(timeout.code=="ANDROID_REQUEST_TIMED_OUT_RESULT_UNKNOWN");
  android_no_actuation(timeout);
  assert(screen.observe_backend(first.sequence,android_controls_accepted(),102.2).code==
         "ANDROID_REPLY_UNMATCHED_OR_REPLAYED");
  auto recent=android_controls_fixture();recent.mission_status_received_sec=102.2;
  screen.show(recent);
  auto second=screen.propose(AndroidMissionIntent::start,102.3);
  assert(second.prepared && second.sequence>first.sequence);
  auto age=screen.observe_backend(second.sequence,android_controls_accepted(),104.31);
  assert(age.code=="ANDROID_REPLY_EXPIRED_RESULT_UNKNOWN");
  auto fresh=android_controls_fixture();fresh.mission_status_received_sec=104.4;
  screen.show(fresh);
  assert(screen.propose(AndroidMissionIntent::start,104.5).prepared);
}
