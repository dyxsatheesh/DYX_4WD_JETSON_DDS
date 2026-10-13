#pragma once
// Phase 2F1: ROS-free PX4 telemetry frame and freshness validation.
// No motor / Offboard / trajectory publishers belong in this module.
#include "dyx_rpp_cpp17_core/primitives.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace dyx::rpp {

struct ShadowTelemetry {
  bool have_position{false};
  bool have_rate{false};
  bool xy_valid{false};
  bool v_xy_valid{false};
  bool heading_good_for_control{false};
  double position_age_sec{0.0};
  double rate_age_sec{0.0};
  std::uint64_t position_timestamp_sample_us{0};
  std::uint64_t rate_timestamp_sample_us{0};
  double position_north_m{0.0};
  double position_east_m{0.0};
  double velocity_north_mps{0.0};
  double velocity_east_mps{0.0};
  double heading_ned_rad{0.0};
  double body_z_rate_frd_radps{0.0};
};

struct ShadowTelemetryResult {
  bool valid{false};
  std::string reason{"UNINITIALIZED"};
  double east_m{0.0};
  double north_m{0.0};
  double yaw_enu_rad{0.0};
  double approximate_yaw_rate_enu_radps{0.0};
  double horizontal_speed_mps{0.0};
};

// PX4 NED heading: 0=North, clockwise positive.
// RPP ENU yaw: 0=East, counter-clockwise positive.
inline double px4_ned_heading_to_enu_yaw(double heading_ned_rad) {
  return wrap_angle(pi * 0.5 - heading_ned_rad);
}

// VehicleAngularVelocity.xyz[2] is BODY-FRD z rate, not earth-frame yaw.
// This negative sign is a NEAR-LEVEL preview approximation only. Physical
// yaw-rate authority needs calibrated frames, full attitude, and timing proof.
inline double near_level_body_z_to_enu_yaw_rate(double body_z_rate_frd_radps) {
  if (!std::isfinite(body_z_rate_frd_radps)) throw std::invalid_argument("body z rate must be finite");
  return -body_z_rate_frd_radps;
}

inline ShadowTelemetryResult inspect_shadow_telemetry(
    const ShadowTelemetry &s, double max_age_sec = 0.25,
    double max_sample_skew_sec = 0.10) {
  if (!std::isfinite(max_age_sec) || max_age_sec <= 0.0 ||
      !std::isfinite(max_sample_skew_sec) || max_sample_skew_sec < 0.0) {
    throw std::invalid_argument("invalid shadow freshness/skew limits");
  }
  if (!s.have_position || !s.have_rate) return {false, "MISSING_TELEMETRY"};
  if (!std::isfinite(s.position_age_sec) || !std::isfinite(s.rate_age_sec) ||
      s.position_age_sec < 0.0 || s.rate_age_sec < 0.0 ||
      s.position_age_sec > max_age_sec || s.rate_age_sec > max_age_sec)
    return {false, "STALE_TELEMETRY"};
  if (!s.xy_valid || !s.v_xy_valid || !s.heading_good_for_control)
    return {false, "INVALID_EKF_LOCAL_STATE"};
  if (s.position_timestamp_sample_us == 0 || s.rate_timestamp_sample_us == 0)
    return {false, "MISSING_PX4_SAMPLE_TIMESTAMP"};
  const auto diff = s.position_timestamp_sample_us > s.rate_timestamp_sample_us
      ? s.position_timestamp_sample_us - s.rate_timestamp_sample_us
      : s.rate_timestamp_sample_us - s.position_timestamp_sample_us;
  // Compare in seconds after integer subtraction, avoiding uint64 overflow.
  if (static_cast<double>(diff) * 1.0e-6 > max_sample_skew_sec)
    return {false, "PX4_SAMPLE_TIME_SKEW"};
  for (const double v : {s.position_north_m, s.position_east_m,
       s.velocity_north_mps, s.velocity_east_mps, s.heading_ned_rad,
       s.body_z_rate_frd_radps}) {
    if (!std::isfinite(v)) return {false, "NONFINITE_TELEMETRY"};
  }
  return {true, "READY", s.position_east_m, s.position_north_m,
          px4_ned_heading_to_enu_yaw(s.heading_ned_rad),
          near_level_body_z_to_enu_yaw_rate(s.body_z_rate_frd_radps),
          std::hypot(s.velocity_north_mps, s.velocity_east_mps)};
}

}  // namespace dyx::rpp
