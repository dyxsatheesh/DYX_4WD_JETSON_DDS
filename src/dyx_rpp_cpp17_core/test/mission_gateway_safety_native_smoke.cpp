#include "mission_command_gateway_fixture.hpp"
int main(){
  NativeMissionCommandGateway g;
  gateway_load(g);
  gateway_feed(g,0); gateway_tick(g,0);
  assert(g.dispatch(gateway_request(ClientMissionAction::start,1,0.1)).accepted());
  gateway_feed(g,0.2); gateway_tick(g,0.2);
  // Independently observed hazard immediately revokes underlying gate.
  assert(g.observe_obstacle({0.21,false})==DdsTelemetryDecision::accepted);
  assert(g.dispatch(gateway_request(ClientMissionAction::skip,2,0.22)).status==
         ClientCommandStatus::runtime_rejected);
  auto blocked=gateway_tick(g,0.22);
  assert(blocked.decision.safety.hold==SafetyHold::obstacle_detected);
  assert(blocked.decision.mission.stage==MissionStage::paused);
  assert(g.dispatch(gateway_request(ClientMissionAction::resume,3,0.23)).status==
         ClientCommandStatus::runtime_rejected);
  auto stop=gateway_request(ClientMissionAction::stop,4,-1.0);
  stop.authorized_control_lease=false; // STOP bypasses lease/RTK/offboard gate.
  assert(g.dispatch(stop).accepted());
  assert(g.mission().stage()==MissionStage::stopped);
  assert(g.safe_stop_requested());
}
