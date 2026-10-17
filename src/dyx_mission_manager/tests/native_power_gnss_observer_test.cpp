#include "dyx_mission_manager/native_power_gnss_observer.hpp"
#include <cassert>
#include <iostream>
using dyx::http_backend::NativePowerGnssObserver;
int main() {
  NativePowerGnssObserver x;
  assert(!x.battery_fresh(1) && !x.rtk_fixed(1));
  x.observe_battery(1,100,25.5,true,false,8,0);
  assert(x.battery_healthy(1.5));
  assert(!x.battery_fresh(2.01));
  x.observe_gps(1,100,5,false,false);
  assert(!x.rtk_fixed(1));
  x.observe_gps(1.1,101,6,false,false);
  assert(x.rtk_fixed(1.2) && !x.corrections_fresh_verified());
  x.observe_gps(1.3,102,6,true,false);
  assert(!x.rtk_fixed(1.3));
  x.observe_battery(2,101,21.9,true,false,8,0);
  assert(!x.battery_healthy(2));
  for (int i=1; i<300; ++i)
    x.observe_battery(2+i*0.1,101+i,21.8,true,false,8,0);
  assert(!x.low_voltage_latched());
  x.observe_battery(32.01,401,21.8,true,false,8,0);
  assert(x.low_voltage_latched());
  x.observe_battery(33,402,25.5,true,false,8,0);
  assert(x.low_voltage_latched() && !x.battery_healthy(33));
  NativePowerGnssObserver y;
  y.observe_battery(1,100,25.5,true,false,8,0);
  y.observe_battery(2,100,25.5,true,false,8,0);
  assert(y.battery_fault() && !y.battery_fresh(2));
  NativePowerGnssObserver z;
  z.observe_battery(1,100,25.5,true,false,6,0);
  assert(!z.battery_healthy(1));
  z.observe_gps(1,100,6,false,false);
  z.observe_gps(2,100,6,false,false);
  assert(z.gps_fault() && !z.rtk_fixed(2));
  std::cout << "PASS: missing/stale/out-of-order battery/GPS rejected; 30s voltage latch; RTK FIX independent of RTCM evidence; no recovery by voltage bounce\n";
}
