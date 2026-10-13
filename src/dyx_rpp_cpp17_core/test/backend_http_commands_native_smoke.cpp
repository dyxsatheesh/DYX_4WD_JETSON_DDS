#include "backend_http_fixture.hpp"
int main() {
  NativeMissionCommandGateway g;
  http_load(g);
  NativeBackendMissionApi commands(g);
  NativeBackendLiveStatus live(g);
  NativeBackendHttpWire wire(commands,live);
  auto status=wire.handle(http_read(kBackendMissionStatusPath,1.0));
  assert(status.status==200 && status.body.find("\"stage\":\"ready\"")!=std::string::npos);
  assert(status.body.find("\"loaded\":true")!=std::string::npos);
  auto bad=wire.handle(http_post(kBackendMissionStartPath,1,1.0));
  assert(bad.status==412 && bad.body.find("FRESH_SAFETY_TICK_REQUIRED")!=std::string::npos);
  backend_live_feed(g,2.0); backend_live_tick(g,2.0);
  auto started=wire.handle(http_post(kBackendMissionStartPath,2,2.01));
  assert(started.status==200 && started.body.find("\"stage\":\"approaching\"")!=std::string::npos);
  assert(started.body.find("\"hardware_command_emitted\":false")!=std::string::npos);
  assert(wire.handle(http_post(kBackendMissionStartPath,2,2.02)).status==409);
  // STOP is authenticated, but needs neither a control lease nor valid time.
  auto stop=http_post(kBackendMissionStopPath,3,-1.0);
  stop.principal.owns_control_lease=false;
  auto stopped=wire.handle(stop);
  assert(stopped.status==200 && stopped.body.find("\"safe_stop_intent\":true")!=std::string::npos);
  assert(g.mission().stage()==MissionStage::stopped);
  assert(!g.emits_hardware_commands());
  assert(wire.handle(http_read(kBackendMissionStatusPath,5.0)).status==200);
}
