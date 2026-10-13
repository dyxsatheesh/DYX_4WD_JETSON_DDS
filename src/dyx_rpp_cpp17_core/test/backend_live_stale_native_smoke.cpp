#include "backend_live_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendLiveStatus live(gateway);
  backend_live_feed(gateway,100.0);
  assert(live.observe_tick(backend_live_tick(gateway,100.0)));
  assert(live.observe_satellites({100.0,0x1234,10}));
  auto r=live.handle(backend_live_req(101.0));
  assert(r.http_status==200 && !r.snapshot_fresh);
  assert(!r.gps_valid && !r.battery_valid && !r.satellites_valid);
  assert(r.rtk_status=="UNAVAILABLE" && !r.safety_hold_fresh);
  assert(r.code=="LIVE_STATUS_STALE_OR_FAULT");
  r=live.handle(backend_live_req(100.01));
  assert(r.snapshot_fresh && r.satellites_valid);
  assert(!live.observe_tick(backend_live_tick(gateway,100.0)));
  r=live.handle(backend_live_req(100.01));
  assert(r.telemetry_integrity_fault && !r.snapshot_fresh);
  assert(!r.gps_valid && !r.satellites_valid);
}
