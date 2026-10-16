#include "dds_mission_runtime_fixture.hpp"

int main() {
  NativeDdsMissionRuntime r;
  load_two_markers(r);
  feed_good(r,0.0);
  r.step(0.0,true,true,false,simple_mission_frame(0.0));
  assert(r.start(0.1));
  feed_good(r,0.2);
  assert(r.step(0.2,true,true,false,simple_mission_frame(0.2)).preview_forward_permitted);
  for (int sec=1;sec<=5;++sec) {
    const auto t=static_cast<double>(sec);
    feed_good(r,t,5);
    const auto observed=r.step(t,true,true,false,simple_mission_frame(t));
    assert(observed.decision.safety.rtk_float_confirming);
    assert(observed.decision.mission.stage==MissionStage::paused);
  }
  feed_good(r,6.0,5);
  const auto hold=r.step(6.0,true,true,false,simple_mission_frame(6.0));
  assert(hold.decision.safety.hold==SafetyHold::rtk_float_paused);
  assert(hold.decision.mission.stage==MissionStage::paused);
  assert(!hold.decision.float_pause_owned);
  assert(!hold.preview_forward_permitted);
  assert(!r.resume_manually(6.1));
  feed_good(r,6.5,5);
  const auto still=r.step(6.5,true,true,false,simple_mission_frame(6.5));
  assert(still.decision.mission.stage==MissionStage::paused);
  feed_good(r,7.0,6);
  const auto recovered=r.step(7.0,true,true,false,simple_mission_frame(7.0));
  assert(!recovered.decision.auto_resumed_float);
  assert(recovered.decision.safety.motion_allowed);
  assert(recovered.decision.mission.stage==MissionStage::paused);
  assert(!recovered.preview_forward_permitted);
  assert(!recovered.physical_command_emitted);

  // FIXED recovery alone cannot resume the mission.
  assert(!r.resume_manually(7.30)); // expired safety evidence
  assert(r.resume_manually(7.10));  // explicit operator action

  feed_good(r,7.2,6);
  const auto resumed=r.step(
      7.2,true,true,false,simple_mission_frame(7.2));
  assert(resumed.decision.mission.stage==MissionStage::approaching);
  assert(resumed.preview_forward_permitted);
  assert(!resumed.physical_command_emitted);
}
