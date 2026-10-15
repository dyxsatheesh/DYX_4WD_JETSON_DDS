#include "backend_api_fixture.hpp"
int main() {
  NativeMissionCommandGateway g;
  NativeBackendMissionApi api(g);
  gateway_load(g,MissionMode::manual);
  gateway_feed(g,1.0);gateway_tick(g,1.0);
  auto start=api.handle(api_req(kBackendMissionStartPath,1,1.1));
  assert(start.http_status==200 && start.stage==MissionStage::approaching);
  assert(!start.hardware_command_emitted);
  assert(api.handle(api_req(kBackendMissionSkipPath,2,1.11)).http_status==412);
  assert(api.handle(api_req(kBackendMissionSkipPath,2,1.12)).http_status==409);
  gateway_feed(g,1.2); gateway_tick(g,1.2);
  auto skip=api.handle(api_req(kBackendMissionSkipPath,3,1.21));
  assert(skip.http_status==200 && skip.skipped==1 && skip.pending==1);
  gateway_feed(g,1.3); gateway_tick(g,1.3);
  assert(api.handle(api_req(kBackendMissionNextPath,4,1.31)).http_status==200);
  auto hold=api.handle(api_req(kBackendMissionPausePath,5,1.32));
  assert(hold.http_status==200 && hold.stage==MissionStage::paused);
  assert(api.handle(api_req(kBackendMissionResumePath,6,1.33)).http_status==412);
  gateway_feed(g,1.4);gateway_tick(g,1.4);
  assert(api.handle(api_req(kBackendMissionResumePath,7,1.41)).http_status==200);
  auto stop=api_req(kBackendMissionStopPath,8,-1.0);
  stop.principal.owns_control_lease=false;
  const auto stopped=api.handle(stop);
  assert(stopped.http_status==200 && stopped.safe_stop_intent &&
         stopped.stage==MissionStage::stopped && !stopped.hardware_command_emitted);

  // H5L: API responses must explicitly report a latched recovery quarantine.
  // No mission is loaded, so other preflight checks must not hide the result.
  NativeMissionCommandGateway qgateway;
  NativeBackendMissionApi qapi(qgateway);
  qgateway.latch_recovery_quarantine();
  auto rejected=qapi.handle(api_req(kBackendMissionStartPath,1,2.0));
  assert(rejected.http_status==423 && rejected.code=="MISSION_RECOVERY_QUARANTINED");
  assert(rejected.decision_only && !rejected.hardware_command_emitted);

  // Mission status stays read-only, and authenticated STOP is still accepted
  // without a control lease or a valid monotonic timestamp.
  auto qstatus=qapi.handle(api_status_req());
  assert(qstatus.http_status==200 && qstatus.decision_only);
  auto qstop=api_req(kBackendMissionStopPath,2,-1.0);
  qstop.principal.owns_control_lease=false;
  auto qstopped=qapi.handle(qstop);
  assert(qstopped.http_status==200 && qstopped.safe_stop_intent);
  assert(!qstopped.hardware_command_emitted);
}
