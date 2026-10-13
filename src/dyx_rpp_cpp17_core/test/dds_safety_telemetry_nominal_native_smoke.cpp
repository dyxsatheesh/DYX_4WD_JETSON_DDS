#include "dyx_rpp_cpp17_core/dds_safety_telemetry.hpp"
#include <cassert>
using namespace dyx::rpp;
int main() {
  NativeDdsSafetyTelemetry telemetry;
  assert(telemetry.observe_vehicle({1000000, 10.0, 2, 14, 3, false, 0}) ==
         DdsTelemetryDecision::accepted);
  assert(telemetry.observe_battery({1000000, 10.0, 24.5, true, false, 8, 0}) ==
         DdsTelemetryDecision::accepted);
  assert(telemetry.observe_gps({1000000, 10.0, 0x123, 12.5, 77.5, 6, false, 0}) ==
         DdsTelemetryDecision::accepted);
  assert(telemetry.observe_estimator({1000000, 10.0, true, true, true, false,
                                     0.2, 1.2, 0.5}) == DdsTelemetryDecision::accepted);
  assert(telemetry.observe_corrections({10.0, 0.2, true}) == DdsTelemetryDecision::accepted);
  assert(telemetry.observe_obstacle({10.0, true}) == DdsTelemetryDecision::accepted);
  auto snap = telemetry.snapshot(10.1, true, true, false);
  assert(!snap.telemetry_integrity_fault && snap.px4_vehicle_fresh);
  assert(snap.gps_sample_fresh && snap.battery_sample_fresh);
  assert(snap.gps_source_selected && snap.gps_device_id == 0x123);
  assert(snap.safety.gnss_fix_type == 6 && snap.safety.gnss_sample_time_sec == 10.0);
  assert(snap.safety.px4_offboard_armed && snap.safety.estimator_valid);
  assert(snap.safety.correction_stream_healthy);
  assert(snap.safety.correction_age_sec > 0.29 && snap.safety.correction_age_sec < 0.31);
  assert(snap.safety.obstacle_clear);
  NativeSafetyArbiter arbiter;
  const auto result = arbiter.step(snap.safety);
  assert(result.motion_allowed && result.hold == SafetyHold::none);
  assert(result.start_resume_next_skip_allowed);
  assert(result.rc_allowed);
}
