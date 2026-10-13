#include "backend_live_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  backend_live_load(gateway);
  NativeBackendLiveStatus live(gateway);
  backend_live_feed(gateway, 100.0);
  assert(live.observe_tick(backend_live_tick(gateway,100.0)));
  assert(live.observe_satellites({100.0, 0x1234, 16}));
  assert(live.observe_battery_remaining({100.0,25.2,0.83}));
  const auto response=live.handle(backend_live_req(100.01));
  assert(response.http_status==200 && response.read_only);
  assert(response.snapshot_fresh && response.gps_valid);
  assert(response.rtk_status=="FIXED" && response.satellites_valid);
  assert(response.satellites_used==16);
  assert(response.battery_valid && response.battery_voltage_v==25.2);
  assert(response.battery_remaining_valid && response.battery_remaining_percent>82.9);
  assert(response.total==2 && response.pending==2);
  assert(response.stage==MissionStage::ready && response.marking_label=="P0001");
  assert(!response.marking_accuracy_valid && response.marking_radial_error_m==0.0);
  assert(!response.hardware_command_emitted);
  assert(gateway.mission().stage()==MissionStage::ready);
}
