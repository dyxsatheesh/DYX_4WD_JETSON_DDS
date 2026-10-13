#include "dyx_rpp_cpp17_core/tracking_stability.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace dyx::rpp;

static void require(bool ok) {
  if (!ok) throw std::runtime_error("tracking stability smoke check failed");
}
int main() {
  TrackingStabilityController c;
  TrackingControlInput s{};
  s.path_identity = "mission-a";
  s.telemetry_fresh = true;
  s.dt_sec = 0.1;
  auto r = c.step(s);
  require(r.valid && r.reset_applied && !r.acceleration_allowed);
  require(r.transition_reason == "path_reset_recapture");
  for (int i = 0; i < 2; ++i) r = c.step(s);
  require(r.state == TrackingControlState::tracking && r.acceleration_allowed);
  s.signed_cross_track_m = 0.06;
  r = c.step(s);
  require(r.state == TrackingControlState::recovery);
  require(r.transition_reason == "recovery_entered:cross_track");
  s.signed_cross_track_m = 0.0;
  r = c.step(s);
  require(r.state == TrackingControlState::recapture_stable);
  s.path_identity = "mission-b";
  r = c.step(s);
  require(r.reset_applied && !r.acceleration_allowed);
  s.telemetry_fresh = false;
  r = c.step(s);
  require(!r.valid && r.invalid_reason == "stale_telemetry");
  require(r.recovery_speed_scale == 0.0);
  s.telemetry_fresh = true;
  s.dt_sec = std::numeric_limits<double>::quiet_NaN();
  r = c.step(s);
  require(!r.valid && r.invalid_reason == "nonfinite_dt_sec");
  s.dt_sec = 0.1;
  s.heading_error_rad = tracking_pi/12.0;
  s.bearing_clamped = true;
  r = c.step(s);
  require(r.transition_reason == "recovery_entered:heading+bearing_clamp");
  return 0;
}
