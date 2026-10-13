#include "backend_live_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendLiveStatus live(gateway);
  backend_live_feed(gateway,100.0);
  assert(live.observe_tick(backend_live_tick(gateway,100.0)));
  assert(live.observe_satellites({100.0,0x9999,9})); // wrong GPS source
  assert(live.observe_battery_remaining({100.0,25.8,0.6})); // wrong voltage
  auto r=live.handle(backend_live_req(100.01));
  assert(r.gps_valid && r.rtk_status=="FIXED");
  assert(!r.satellites_valid && !r.battery_remaining_valid);
  assert(!live.observe_satellites({100.0,0x1234,18})); // old stamp cannot overwrite
  assert(!live.observe_battery_remaining({100.0,25.2,0.6})); // replay
  backend_live_feed(gateway,100.1,5);
  assert(live.observe_tick(backend_live_tick(gateway,100.1)));
  r=live.handle(backend_live_req(100.11));
  assert(r.rtk_status=="FLOAT" && !r.satellites_valid);
  assert(!r.battery_remaining_valid);
  assert(!live.observe_satellites({100.1,0x1234,129}));
  assert(!live.observe_battery_remaining({100.1,25.2,1.1}));
  assert(live.observe_satellites({100.1,0x1234,12}));
  assert(live.observe_battery_remaining({100.1,25.2,0.42}));
  r=live.handle(backend_live_req(100.11));
  assert(r.satellites_valid && r.satellites_used==12);
  assert(r.battery_remaining_valid && r.battery_remaining_percent>41.9);
}
