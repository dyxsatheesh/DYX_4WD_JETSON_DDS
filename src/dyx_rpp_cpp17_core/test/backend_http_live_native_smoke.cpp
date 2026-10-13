#include "backend_http_fixture.hpp"
int main() {
  NativeMissionCommandGateway g;
  http_load(g);
  NativeBackendMissionApi commands(g);
  NativeBackendLiveStatus live(g);
  NativeBackendHttpWire wire(commands, live);
  backend_live_feed(g, 10.0);
  assert(live.observe_tick(backend_live_tick(g,10.0)));
  assert(live.observe_satellites({10.0,0x1234,15}));
  assert(live.observe_battery_remaining({10.0,25.2,0.72}));
  const auto r=wire.handle(http_read(kBackendLiveStatusPath,10.02));
  assert(r.status==200 && !r.physical_command_emitted && !r.network_io_performed);
  assert(r.body.find("\"rtk\":\"FIXED\"")!=std::string::npos);
  assert(r.body.find("\"satellites_used\":15")!=std::string::npos);
  assert(r.body.find("\"battery_voltage_v\":25.2")!=std::string::npos);
  assert(r.body.find("\"battery_remaining_percent\":72")!=std::string::npos);
  assert(r.body.find("\"marking_accuracy_m\":null")!=std::string::npos);
  assert(r.body.find("\"pending\":2")!=std::string::npos);
  const auto http=r.http11_bytes();
  assert(http.find("HTTP/1.1 200 OK\r\n")==0);
  assert(http.find("Cache-Control: no-store\r\n")!=std::string::npos);
  assert(http.find("X-Content-Type-Options: nosniff\r\n")!=std::string::npos);
  assert(http.find("Content-Length: "+std::to_string(r.body.size())+"\r\n")!=std::string::npos);
  assert(http.substr(http.find("\r\n\r\n")+4)==r.body);
  assert(g.mission().stage()==MissionStage::ready);
  const auto event=wire.make_live_sse_frame(http_read(kBackendLiveStatusPath,10.02),17);
  assert(event=="id: 17\nevent: rover.status\ndata: "+r.body+"\n\n");
  assert(wire.make_live_sse_frame(http_read(kBackendLiveStatusPath,10.02),0).empty());
}
