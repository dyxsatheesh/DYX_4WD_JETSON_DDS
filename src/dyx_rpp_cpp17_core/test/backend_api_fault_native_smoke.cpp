#include "backend_api_fixture.hpp"
int main() {
  NativeMissionCommandGateway g;
  NativeBackendMissionApi api(g);
  gateway_load(g);
  gateway_feed(g,1.0);gateway_tick(g,1.0);
  assert(api.handle(api_req(kBackendMissionStartPath,1,1.1)).http_status==200);
  gateway_feed(g,1.2);gateway_tick(g,1.2);
  assert(g.observe_obstacle({1.21,false})==DdsTelemetryDecision::accepted);
  const auto denied=api.handle(api_req(kBackendMissionSkipPath,2,1.22));
  assert(denied.http_status==412 && !denied.hardware_command_emitted);
  gateway_tick(g,1.22);
  auto snap=api.handle(api_status_req());
  assert(snap.stage==MissionStage::paused);
  assert(snap.reason=="EXPLICIT_PAUSE");
  // PX4 time reset triggers fail-closed stop before next API command.
  assert(g.observe_vehicle({gateway_stamp(1.2)-1,1.3,2,14,3,false,0})==
         DdsTelemetryDecision::px4_epoch_changed);
  const auto stopped=api.handle(api_status_req());
  assert(stopped.safe_stop_intent && stopped.stage==MissionStage::stopped);
  assert(!stopped.mission_loaded);
  assert(api.handle(api_req(kBackendMissionStartPath,3,1.31)).http_status==409);
  const auto stillstop=api.handle(api_req(kBackendMissionStopPath,4,-1.0));
  assert(stillstop.http_status==200 && stillstop.safe_stop_intent);
}
