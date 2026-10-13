#include "backend_live_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  backend_live_load(gateway);
  NativeBackendLiveStatus live(gateway);
  backend_live_feed(gateway,100.0);
  assert(live.observe_tick(backend_live_tick(gateway,100.0)));
  auto req=backend_live_req(100.01);
  req.principal.authenticated=false;
  auto r=live.handle(req);
  assert(r.http_status==401 && r.total==0 && !r.battery_valid && !r.mission_loaded);
  req.principal.authenticated=true;
  req.principal.can_read_mission=false;
  r=live.handle(req);
  assert(r.http_status==403 && r.total==0 && !r.mission_loaded);
  req.principal.can_read_mission=true;
  req.method="POST";
  r=live.handle(req);
  assert(r.http_status==405 && r.total==0);
  req.method="GET";req.path="/api/v1/mission/start";
  r=live.handle(req);
  assert(r.http_status==404 && r.total==0);
  req.path=kBackendLiveStatusPath;
  req.host_now_sec=std::numeric_limits<double>::quiet_NaN();
  r=live.handle(req);
  assert(r.http_status==400 && r.total==0);
  assert(gateway.mission().stage()==MissionStage::ready);
}
