#include "dyx_rpp_cpp17_core/dds_safety_telemetry.hpp"
#include <cassert>
using namespace dyx::rpp;
int main() {
  NativeDdsSafetyTelemetry t;
  assert(t.observe_gps({1000000, 10, 2, 1.0, 2.0, 6, false, 0}) == DdsTelemetryDecision::accepted);
  assert(t.observe_gps({990000, 11, 2, 1.0, 2.0, 6, false, 0}) == DdsTelemetryDecision::px4_epoch_changed);
  assert(t.integrity_fault_latched());
  assert(t.observe_vehicle({100, 12, 2, 14, 3, false, 0}) == DdsTelemetryDecision::fault_latched);
  auto s = t.snapshot(12, true, true, false);
  assert(s.telemetry_integrity_fault && !s.safety.px4_offboard_armed);
  assert(s.safety.gnss_sample_time_sec < 0);
  assert(!t.begin_new_disarmed_px4_epoch(false, true));
  assert(!t.begin_new_disarmed_px4_epoch(true, false));
  assert(t.begin_new_disarmed_px4_epoch(true, true));
  assert(!t.integrity_fault_latched());
  auto cleared = t.snapshot(12, true, true, false);
  assert(!cleared.telemetry_integrity_fault);
  assert(cleared.safety.gnss_sample_time_sec < 0 && !cleared.safety.px4_offboard_armed);
  assert(t.observe_gps({100, 13, 3, 1.0, 2.0, 6, false, 0}) == DdsTelemetryDecision::accepted);
  assert(t.snapshot(13, true, true, false).gps_device_id == 3);
}
