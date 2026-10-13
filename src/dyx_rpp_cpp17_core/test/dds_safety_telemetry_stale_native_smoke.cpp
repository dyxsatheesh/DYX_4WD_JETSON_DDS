#include "dyx_rpp_cpp17_core/dds_safety_telemetry.hpp"
#include <cassert>
#include <limits>
using namespace dyx::rpp;
int main() {
  NativeDdsSafetyTelemetry t;
  auto s = t.snapshot(4.0, true, true, false);
  assert(!s.safety.px4_offboard_armed && !s.safety.estimator_valid);
  assert(!s.safety.correction_stream_healthy && !s.safety.obstacle_clear);
  assert(s.safety.battery_sample_time_sec < 0 && s.safety.gnss_sample_time_sec < 0);
  assert(t.observe_gps({100, 5.0, 7, 12.0, 77.0, 6, false, 0}) == DdsTelemetryDecision::accepted);
  auto before = t.snapshot(5.1, true, true, false);
  assert(before.gps_sample_fresh && before.safety.gnss_fix_type == 6);
  // A replay must NOT renew the freshness deadline.
  assert(t.observe_gps({100, 6.0, 7, 12.0, 77.0, 6, false, 0}) == DdsTelemetryDecision::duplicate);
  auto expired = t.snapshot(6.51, true, true, false);
  assert(!expired.gps_sample_fresh && expired.safety.gnss_sample_time_sec < 0);
  assert(expired.safety.gnss_fix_type == 0);
  // Likewise, a correction heartbeat cannot keep old correction age alive.
  assert(t.observe_corrections({6.6, 3.0, true}) == DdsTelemetryDecision::accepted);
  auto c1 = t.snapshot(6.8, true, true, false);
  assert(c1.safety.correction_stream_healthy);
  assert(c1.safety.correction_age_sec > 3.19);
  auto c2 = t.snapshot(7.7, true, true, false);
  assert(!c2.safety.correction_stream_healthy);
  assert(!t.integrity_fault_latched());
}
