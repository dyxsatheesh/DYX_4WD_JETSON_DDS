#include "backend_android_controls_fixture.hpp"
int main() {
  NativeOfflineAndroidMissionControls screen;
  auto a=android_controls_fixture();screen.show(a);
  auto first=screen.propose(AndroidMissionIntent::start,100.1);
  assert(first.prepared);
  a.mission_run_id="new_mission_run";
  screen.show(a);
  assert(!screen.request_pending());
  assert(screen.observe_backend(first.sequence,android_controls_accepted(),100.2).code==
         "ANDROID_REPLY_UNMATCHED_OR_REPLAYED");
  auto second=screen.propose(AndroidMissionIntent::start,100.3);
  assert(second.prepared && second.sequence>first.sequence);
  assert(screen.propose(AndroidMissionIntent::start,90.0).code==
         "ANDROID_HOST_CLOCK_FAULT_LATCHED"); // regression latches even with a pending proposal
  auto stop=screen.propose(AndroidMissionIntent::stop,89.0);
  assert(stop.prepared);
  assert(screen.clock_fault_latched());
  android_no_actuation(stop);
  assert(screen.propose(AndroidMissionIntent::start,100.4).code==
         "ANDROID_HOST_CLOCK_FAULT_LATCHED");
  assert(!screen.recover_disarmed(true,false,101.0));
  assert(screen.recover_disarmed(true,true,101.0));
  screen.show(android_controls_fixture());
  assert(screen.propose(AndroidMissionIntent::start,101.1).sequence>stop.sequence);
}
