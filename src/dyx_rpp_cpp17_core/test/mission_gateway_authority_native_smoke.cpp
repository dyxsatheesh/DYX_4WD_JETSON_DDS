#include "mission_command_gateway_fixture.hpp"
#include <limits>
int main(){
  NativeMissionCommandGateway g;
  assert(g.load(gateway_run,gateway_session,{{0,0,"P0001","RUN:P1"}},
      MissionMode::automatic,false,true,true)==ClientCommandStatus::unauthorized);
  assert(g.load(gateway_run,gateway_session,{{0,0,"P0001","RUN:P1"}},
      MissionMode::automatic,true,true,false)==ClientCommandStatus::unauthorized);
  gateway_load(g);
  gateway_feed(g,0);gateway_tick(g,0);
  auto wrong=gateway_request(ClientMissionAction::start,1,0.1);
  wrong.mission_run_id="another_run_0001";
  assert(g.dispatch(wrong).status==ClientCommandStatus::wrong_run);
  wrong=gateway_request(ClientMissionAction::start,1,0.1);
  wrong.operator_session_id="another_session_0001";
  assert(g.dispatch(wrong).status==ClientCommandStatus::wrong_run);
  wrong=gateway_request(ClientMissionAction::start,1,0.1);
  wrong.authenticated_operator=false;
  assert(g.dispatch(wrong).status==ClientCommandStatus::unauthorized);
  wrong=gateway_request(ClientMissionAction::start,1,0.1);
  wrong.authorized_control_lease=false;
  assert(g.dispatch(wrong).status==ClientCommandStatus::unauthorized);
  assert(g.dispatch(gateway_request(ClientMissionAction::start,1,0.1)).accepted());
  assert(g.dispatch(gateway_request(ClientMissionAction::start,1,0.2)).status==
         ClientCommandStatus::duplicate_or_reordered);
  auto badclock=gateway_request(ClientMissionAction::skip,2,
         std::numeric_limits<double>::quiet_NaN());
  assert(g.dispatch(badclock).status==ClientCommandStatus::invalid_clock);
  assert(g.dispatch(gateway_request(ClientMissionAction::skip,2,0.3)).status==
         ClientCommandStatus::stale_or_consumed_safety);
  // Every consumed or denied well-formed request has a unique increasing seq.
  assert(g.dispatch(gateway_request(ClientMissionAction::skip,2,0.4)).status==
         ClientCommandStatus::duplicate_or_reordered);
  g.force_stop();
  assert(g.safe_stop_requested()&&!g.has_active_mission());
}
