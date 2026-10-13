#include "dyx_rpp_cpp17_core/tracking_metrics.hpp"
#include <cmath>
#include <stdexcept>

using namespace dyx::rpp;

int main() {
  const auto check = [](bool test) {
    if (!test) throw std::runtime_error("tracking metrics assertion failed");
  };
  TrackingMetricsConfig c;
  c.metrics_quantile_window_capacity = 3;
  c.metrics_histogram_bin_width_m = 0.01;
  c.metrics_histogram_max_m = 0.10;
  TrackingMetricsAccumulator a(c);
  TrackingControlInput s{};
  s.path_identity = "path-1";
  s.telemetry_fresh = true;
  s.commanded_speed_mps = 0.8;
  s.measured_speed_mps = 0.6;
  s.dt_sec = 0.2;
  s.projection_s_m = 1.0;
  s.signed_cross_track_m = 0.02;
  check(a.add(s, TrackingControlState::tracking, "mission-1"));
  check(a.snapshot().cruise_time_sec == 0.1);
  check(std::abs(a.snapshot().rms_speed_error_mps - 0.2) < 1e-12);
  s.projection_s_m = 0.0;
  s.signed_cross_track_m = 0.20;
  check(a.add(s, TrackingControlState::recovery, "mission-1"));
  check(a.snapshot().monotonic_s_violation_count == 1);
  check(a.snapshot().histogram_overflow_count == 1);
  check(a.snapshot().p95_histogram_saturated);
  check(a.quantile_storage_size() == 2);
  s.telemetry_fresh = false;
  check(!a.add(s, TrackingControlState::recovery, "mission-1"));
  check(a.snapshot().rejected_sample_count == 1);
  a.note_discontinuity("   ");
  check(a.snapshot().last_discontinuity_reason.value() == "unspecified");
  check(!a.snapshot().valid_for_acceptance);
  s.telemetry_fresh = true;
  s.projection_s_m = 0.1;
  check(a.add(s, TrackingControlState::recapture_stable, "mission-1"));
  check(a.snapshot().monotonic_s_violation_count == 1); // cleared epoch
  check(a.quantile_storage_size() == 3);
  s.signed_cross_track_m = 0.001;
  check(a.add(s, TrackingControlState::tracking, "mission-1"));
  check(a.quantile_storage_size() == 3); // bounded trailing window
  a.reset();
  const auto cleared = a.snapshot();
  check(cleared.sample_count == 0 && cleared.valid_for_acceptance);
  check(cleared.quantile_sample_count == 0 && cleared.discontinuity_count == 0);
  bool rejected_config = false;
  try { TrackingMetricsConfig bad; bad.metrics_histogram_bin_width_m = 1e-9;
        TrackingMetricsAccumulator b(bad); }
  catch (const std::invalid_argument &) { rejected_config = true; }
  check(rejected_config);
  return 0;
}
