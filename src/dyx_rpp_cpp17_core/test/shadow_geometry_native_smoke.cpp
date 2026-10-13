#include "dyx_rpp_cpp17_core/shadow_geometry.hpp"
#include "dyx_rpp_cpp17_core/forward_yaw_rate.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

using namespace dyx::rpp;
static void check(bool v, const char *why) {
  if (!v) throw std::runtime_error(why);
}
static bool near(double a, double b, double tol = 1e-12) {
  return std::abs(a-b) <= tol;
}

int main() {
  // PX4 NED cardinal heading conversion, incl. wrap at +/-180 deg.
  check(near(px4_ned_heading_to_enu_yaw(0), pi/2), "North");
  check(near(px4_ned_heading_to_enu_yaw(pi/2), 0), "East");
  check(near(px4_ned_heading_to_enu_yaw(-pi/2), pi), "West");
  check(near(px4_ned_heading_to_enu_yaw(pi), -pi/2), "South");
  ShadowTelemetry t;
  check(inspect_shadow_telemetry(t).reason == "MISSING_TELEMETRY", "missing");
  t.have_position = t.have_rate = true;
  t.xy_valid = t.v_xy_valid = t.heading_good_for_control = true;
  t.position_timestamp_sample_us = 1000000;
  t.rate_timestamp_sample_us = 1020000;
  t.position_north_m = 10.0;
  t.position_east_m = 20.0;
  t.heading_ned_rad = 0.0;
  t.body_z_rate_frd_radps = 0.02;
  t.velocity_north_mps = 0.6;
  auto r = inspect_shadow_telemetry(t);
  check(r.valid && near(r.east_m,20) && near(r.north_m,10), "NED xy mapping");
  check(near(r.yaw_enu_rad,pi/2) && near(r.approximate_yaw_rate_enu_radps,-0.02), "yaw mapping");
  check(near(r.horizontal_speed_mps,0.6), "speed mapping");
  t.position_age_sec = 0.26;
  check(inspect_shadow_telemetry(t).reason == "STALE_TELEMETRY", "stale fail closed");
  t.position_age_sec = 0.0;
  t.rate_timestamp_sample_us += 200000;
  check(inspect_shadow_telemetry(t).reason == "PX4_SAMPLE_TIME_SKEW", "skew fail closed");
  t.rate_timestamp_sample_us = 1020000;
  t.heading_good_for_control = false;
  check(inspect_shadow_telemetry(t).reason == "INVALID_EKF_LOCAL_STATE", "heading gate");
  t.heading_good_for_control = true;
  t.heading_ned_rad = std::numeric_limits<double>::quiet_NaN();
  check(inspect_shadow_telemetry(t).reason == "NONFINITE_TELEMETRY", "nan gate");
  t.heading_ned_rad = 0.0;

  // Pure preview cannot send anything to PX4; verify the planner's output is
  // forward in the instantaneous body-X direction for 4 orientations.
  for (double yaw : {-2.7, -pi/2, 0.0, pi/2, 2.8}) {
    ForwardYawRatePlanner planner;
    ForwardYawInput s;
    s.now_sec = 1.0;
    s.dt_sec = 0.02;
    s.rover_yaw_enu_rad = yaw;
    s.target_yaw_enu_rad = wrap_angle(yaw + 0.1);
    s.requested_forward_speed_mps = 0.60;
    s.motion_enabled = s.telemetry_fresh = true;
    const auto out = planner.step(s);
    check(out.phase == ForwardYawPhase::TRACK, "must track");
    check(out.forward_speed_mps >= 0.0, "no backward preview");
    const double north = std::sin(yaw);
    const double east = std::cos(yaw);
    check(near(out.velocity_north_mps*north + out.velocity_east_mps*east, 0.6,1e-10),
          "forward projection");
  }
  return 0;
}
