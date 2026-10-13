#include "dds_mission_runtime_fixture.hpp"

int main() {
  NativeDdsMissionRuntime r;
  load_two_markers(r);
  feed_good(r,0.0);
  const auto okay=r.step(0.0,true,true,false,simple_mission_frame(0.0));
  assert(okay.decision.safety.motion_allowed);
  // Even a duplicate ingress revokes previously sampled START authority.
  assert(r.observe_vehicle({fake_px4_stamp(0.0),0.0,2,14,3,false,0})==
         DdsTelemetryDecision::duplicate);
  assert(!r.start(0.1));
  assert(r.step(0.1,true,true,false,simple_mission_frame(0.1)).decision.safety.motion_allowed);
  assert(r.start(0.12));
  feed_good(r,0.2);
  assert(r.step(0.2,true,true,false,simple_mission_frame(0.2)).preview_forward_permitted);
  // Backwards PX4 boot-time stamp means a new boot epoch or disorder:
  // stop and latch, never auto-resume after a time reset.
  assert(r.observe_vehicle({fake_px4_stamp(0.1),0.3,2,14,3,false,0})==
         DdsTelemetryDecision::px4_epoch_changed);
  assert(r.telemetry_fault_latched());
  assert(r.mission().stage()==MissionStage::stopped);
  assert(!r.resume_manually(0.31));
  assert(r.begin_new_disarmed_px4_epoch(true,true));
  load_two_markers(r);
  feed_good(r,1.0);
  // Reject mission evidence created from a different clock/tick.
  const auto mismatched=r.step(1.0,true,true,false,simple_mission_frame(0.8));
  assert(mismatched.decision.mission.stage==MissionStage::stopped);
  assert(!mismatched.preview_forward_permitted);
  assert(!r.start(1.1));
}
