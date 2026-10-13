#include "dds_mission_runtime_fixture.hpp"

int main() {
  NativeDdsMissionRuntime r;
  load_two_markers(r);
  assert(!r.start(0.0)); // no safety evidence
  feed_good(r,0.0);
  const auto loaded=r.step(0.0,true,true,false,simple_mission_frame(0.0));
  assert(!loaded.telemetry.telemetry_integrity_fault);
  assert(!loaded.telemetry_fault_latched && !loaded.physical_command_emitted);
  assert(loaded.decision.safety.motion_allowed);
  assert(loaded.decision.mission.stage==MissionStage::ready);
  assert(!loaded.preview_forward_permitted); // mission has not STARTed
  assert(r.start(0.1));
  feed_good(r,0.2);
  const auto tracking=r.step(0.2,true,true,false,simple_mission_frame(0.2));
  assert(tracking.preview_forward_permitted);
  assert(tracking.decision.mission.stage==MissionStage::approaching);
  assert(!tracking.physical_command_emitted);
  assert(r.observe_obstacle({0.21,false})==DdsTelemetryDecision::accepted);
  // A fresh danger update revokes old command authority BEFORE the next tick.
  assert(!r.skip(0.22));
  const auto obstacle=r.step(0.21,true,true,false,simple_mission_frame(0.21));
  assert(!obstacle.preview_forward_permitted);
  assert(obstacle.decision.safety.hold==SafetyHold::obstacle_detected);
  assert(obstacle.decision.mission.stage==MissionStage::paused);
  r.stop();
  assert(!r.has_active_mission());
  assert(!r.start(0.23));
}
