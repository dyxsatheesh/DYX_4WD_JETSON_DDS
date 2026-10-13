#include "dyx_rpp_cpp17_core/dds_safety_telemetry.hpp"
#include <cassert>
#include <limits>
using namespace dyx::rpp;
int main() {
  NativeDdsSafetyTelemetry t;
  // PX4 warning is an immediate battery-health hold independent of the
  // separate 30-second voltage-only debounce policy.
  assert(t.observe_battery({100, 1, 24, true, false, 8, 2}) == DdsTelemetryDecision::accepted);
  auto bad_battery = t.snapshot(1.1, true, true, false);
  assert(bad_battery.battery_sample_fresh);
  assert(bad_battery.safety.battery_sample_time_sec < 0);
  assert(t.observe_vehicle({200, 1.2, 2, 14, 3, true, 0}) == DdsTelemetryDecision::accepted);
  assert(!t.snapshot(1.2, true, true, false).safety.px4_offboard_armed);
  assert(t.observe_gps({100, 1.3, 7, 12.5, 77.6, 6, true, 0}) == DdsTelemetryDecision::accepted);
  assert(t.snapshot(1.3, true, true, false).safety.gnss_sample_time_sec < 0);
  assert(t.observe_gps({200, 1.4, 9, 12.5, 77.6, 6, false, 0}) == DdsTelemetryDecision::invalid_data);
  assert(t.snapshot(1.4, true, true, false).telemetry_integrity_fault);
  assert(t.begin_new_disarmed_px4_epoch(true, true));
  assert(t.observe_battery({200, 5.0, std::numeric_limits<double>::quiet_NaN(),
                            true, false, 8, 0}) == DdsTelemetryDecision::invalid_data);
  assert(t.integrity_fault_latched());
  assert(t.begin_new_disarmed_px4_epoch(true, true));
  assert(t.observe_obstacle({7.0, true}) == DdsTelemetryDecision::accepted);
  assert(t.observe_corrections({6.9, 1.0, true}) == DdsTelemetryDecision::clock_regression);
  assert(t.integrity_fault_latched());
}
