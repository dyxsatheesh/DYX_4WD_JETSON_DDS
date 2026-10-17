#include "dyx_mission_manager/native_telemetry_safety_component.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using dyx::http_backend::NativeTelemetrySafetyComponent;
int main() {
  NativeTelemetrySafetyComponent component;
  assert(!component.power().battery_fresh(1));
  assert(!component.rtk().fixed(1));
  assert(!component.corrections_fresh_verified());
  assert(!component.rtcm().receiver_report_connected());
  component.observe_battery(1, 101, 25.5, true, false, 8, 0);
  assert(component.power().battery_healthy(1.2));
  component.observe_gps(1, 101, 5, false, false);
  component.observe_gps(1.1, 102, 6, false, false);
  assert(component.rtk().fixed(1.2));
  assert(component.power().rtk_fixed(1.2));
  assert(!component.corrections_fresh_verified());
  component.observe_gps(2, 103, 5, false, false);
  for (int i=1; i<=52; ++i)
    component.observe_gps(2 + 0.1*i, 103 + i, 5, false, false);
  assert(component.rtk().float_over_five_seconds(7.2));
  assert(component.rtk().pause_requested(7.2));
  component.observe_gps(7.3, 156, 6, false, false);
  assert(component.rtk().fixed(7.3));
  assert(!component.rtk().float_over_five_seconds(7.3));
  assert(!component.corrections_fresh_verified());
  component.observe_battery(8.0, 201, 21.9, true, false, 8, 0);
  for (int i=1; i<=300; ++i)
    component.observe_battery(8.0+0.1*i,201+i,21.9,true,false,8,0);
  assert(component.critical_battery_latched());
  component.observe_battery(39.0, 502, 25.5, true, false, 8, 0);
  assert(component.critical_battery_latched());
  std::cout << "PASS: GNSS/battery observers isolated; FIX/FLOAT, low-voltage latch, RTCM fail-closed\n";
}
