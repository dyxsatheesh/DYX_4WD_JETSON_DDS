#pragma once
#include <algorithm>
#include <cmath>

namespace dyx {

struct NedCommand {
  float north_mps{0.0f};
  float east_mps{0.0f};
  float yaw_rate_ned_radps{0.0f};
  bool valid{false};
};

// Source RppCommand uses North/East components and CCW-positive ENU yaw rate.
// PX4 trajectory_setpoint uses NED North/East and CW-positive yaw rate.
// No swapping of velocity components is needed: only yaw-rate sign flips.
inline NedCommand convert_rpp(double north, double east,
                              double yaw_enu, double yaw_rate_enu,
                              bool yaw_valid, double speed_limit,
                              double yaw_rate_limit) {
  if (!yaw_valid || !std::isfinite(north) || !std::isfinite(east) ||
      !std::isfinite(yaw_enu) || !std::isfinite(yaw_rate_enu) ||
      !std::isfinite(speed_limit) ||
      !std::isfinite(yaw_rate_limit) || speed_limit <= 0.0 ||
      yaw_rate_limit <= 0.0) {
    return {};
  }

  const double speed = std::hypot(north, east);
  if (!std::isfinite(speed)) {
    return {};
  }
  const double scale = speed > speed_limit ? speed_limit / speed : 1.0;
  const double bounded_yaw_enu =
      std::clamp(yaw_rate_enu, -yaw_rate_limit, yaw_rate_limit);
  return {static_cast<float>(north * scale),
          static_cast<float>(east * scale),
          static_cast<float>(-bounded_yaw_enu), true};
}

} // namespace dyx
