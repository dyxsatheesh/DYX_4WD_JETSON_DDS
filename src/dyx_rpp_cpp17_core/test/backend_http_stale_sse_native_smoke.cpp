#include "backend_http_fixture.hpp"
int main() {
  NativeMissionCommandGateway g;
  http_load(g);
  NativeBackendMissionApi api(g);
  NativeBackendLiveStatus live(g);
  NativeBackendHttpWire wire(api,live);
  backend_live_feed(g,100.0);
  assert(live.observe_tick(backend_live_tick(g,100.0)));
  assert(live.observe_satellites({100.0,0x1234,18}));
  const auto now=wire.handle(http_read(kBackendLiveStatusPath,100.01));
  assert(now.status==200);
  assert(now.body.find("\"rtk\":\"FIXED\"")!=std::string::npos);
  const auto stale=wire.handle(http_read(kBackendLiveStatusPath,101.0));
  assert(stale.status==200);
  assert(stale.body.find("\"fresh\":false")!=std::string::npos);
  assert(stale.body.find("\"rtk\":\"UNAVAILABLE\"")!=std::string::npos);
  assert(stale.body.find("\"satellites_used\":null")!=std::string::npos);
  assert(stale.body.find("\"battery_voltage_v\":null")!=std::string::npos);
  const auto event=wire.make_live_sse_frame(http_read(kBackendLiveStatusPath,101.0),99);
  assert(event.find("id: 99\nevent: rover.status\ndata: {")==0);
  assert(event.find("\"rtk\":\"UNAVAILABLE\"")!=std::string::npos);
  auto forged=http_read(kBackendLiveStatusPath,100.01);
  forged.method="POST";
  assert(wire.make_live_sse_frame(forged,100).empty());
  forged=http_read(kBackendLiveStatusPath,100.01);
  forged.principal.can_read_mission=false;
  assert(wire.make_live_sse_frame(forged,100).empty());
  assert(g.mission().stage()==MissionStage::ready);
}
