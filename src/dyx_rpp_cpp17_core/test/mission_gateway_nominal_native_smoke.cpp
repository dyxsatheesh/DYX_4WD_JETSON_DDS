#include "mission_command_gateway_fixture.hpp"
int main(){
  NativeMissionCommandGateway g;
  gateway_load(g,MissionMode::manual);
  assert(!g.emits_hardware_commands());
  assert(g.dispatch(gateway_request(ClientMissionAction::start,1,0.0)).status==
         ClientCommandStatus::stale_or_consumed_safety);
  gateway_feed(g,1.0);
  auto healthy=gateway_tick(g,1.0);
  assert(healthy.decision.safety.motion_allowed);
  assert(g.dispatch(gateway_request(ClientMissionAction::start,2,1.1)).accepted());
  // The safety frame is consumed by START; SKIP needs a NEW control tick.
  assert(g.dispatch(gateway_request(ClientMissionAction::skip,3,1.11)).status==
         ClientCommandStatus::stale_or_consumed_safety);
  gateway_feed(g,1.2); gateway_tick(g,1.2);
  assert(g.dispatch(gateway_request(ClientMissionAction::skip,4,1.21)).accepted());
  assert(g.mission().stage()==MissionStage::resolved);
  gateway_feed(g,1.3); gateway_tick(g,1.3);
  assert(g.mission().stage()==MissionStage::await_next);
  assert(g.dispatch(gateway_request(ClientMissionAction::next,5,1.31)).accepted());
  assert(g.mission().stage()==MissionStage::approaching);
  assert(g.dispatch(gateway_request(ClientMissionAction::skip,6,1.32)).status==
         ClientCommandStatus::stale_or_consumed_safety);
  const auto stop=g.dispatch(gateway_request(ClientMissionAction::stop,7,1000.0));
  assert(stop.accepted() && stop.safe_stop_intent && !stop.hardware_command_emitted);
  assert(g.mission().stage()==MissionStage::stopped);
  assert(!g.has_active_mission());
}
