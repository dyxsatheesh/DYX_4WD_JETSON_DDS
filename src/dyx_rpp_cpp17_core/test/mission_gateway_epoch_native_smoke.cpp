#include "mission_command_gateway_fixture.hpp"
int main(){
  NativeMissionCommandGateway g;
  gateway_load(g);
  gateway_feed(g,0);gateway_tick(g,0);
  assert(g.dispatch(gateway_request(ClientMissionAction::start,1,0.1)).accepted());
  // A restarted PX4 epoch must stop the mission before a new request.
  assert(g.observe_vehicle({gateway_stamp(0)-1,0.2,2,14,3,false,0})==
         DdsTelemetryDecision::px4_epoch_changed);
  assert(g.safe_stop_requested() && !g.has_active_mission());
  const auto stopped=gateway_tick(g,0.2);
  assert(stopped.telemetry_fault_latched);
  assert(!g.has_active_mission()&&g.safe_stop_requested());
  assert(g.dispatch(gateway_request(ClientMissionAction::start,2,0.21)).status==
         ClientCommandStatus::no_mission);
  assert(!g.begin_new_disarmed_px4_epoch(false,true));
  assert(!g.begin_new_disarmed_px4_epoch(true,false));
  assert(g.begin_new_disarmed_px4_epoch(true,true));
  assert(g.load(gateway_run,gateway_session,{{0,0,"P0001","RUN:P1"}},
      MissionMode::automatic,true,true,true)==ClientCommandStatus::run_id_reused);
  const auto newrun="run_2H4C_0002";
  assert(g.load(newrun,gateway_session,{{0,0,"P0001","RUN:NEW:P1"}},
      MissionMode::automatic,true,true,true)==ClientCommandStatus::accepted);
  assert(!g.safe_stop_requested());
  // Old-run requests cannot alter a newly loaded mission.
  assert(g.dispatch(gateway_request(ClientMissionAction::stop,3,0.3)).status==
         ClientCommandStatus::wrong_run);
  g.force_stop();
  assert(g.mission().stage()==MissionStage::stopped);
}
