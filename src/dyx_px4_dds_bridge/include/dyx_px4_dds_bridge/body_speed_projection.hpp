#pragma once

#include <algorithm>
#include <cmath>

namespace dyx::bridge {

struct BodySpeedProjection {
  float forward_mps{0.0f};
  float lateral_mps{0.0f};
  bool valid{false};
};

// RPP: North/East ground velocity, heading measured CCW from East.
// PX4 differential rover: signed longitudinal/body-forward speed.
inline BodySpeedProjection project_body_speed(
    double north_mps,
    double east_mps,
    double heading_enu_rad,
    bool yaw_valid,
    double max_speed_mps = 1.0,
    double max_lateral_mps = 0.03) noexcept {

  BodySpeedProjection result{};

  if (!yaw_valid ||
      !std::isfinite(north_mps) ||
      !std::isfinite(east_mps) ||
      !std::isfinite(heading_enu_rad) ||
      !std::isfinite(max_speed_mps) ||
      !std::isfinite(max_lateral_mps) ||
      max_speed_mps <= 0.0 ||
      max_lateral_mps < 0.0) {
    return result;
  }

  const double c = std::cos(heading_enu_rad);
  const double s = std::sin(heading_enu_rad);

  const double forward = east_mps * c + north_mps * s;
  const double lateral = -east_mps * s + north_mps * c;

  if (!std::isfinite(forward) ||
      !std::isfinite(lateral) ||
      std::abs(lateral) > max_lateral_mps) {
    return result;
  }

  result.forward_mps = static_cast<float>(
      std::clamp(forward, -max_speed_mps, max_speed_mps));
  result.lateral_mps = static_cast<float>(lateral);
  result.valid = true;
  return result;
}

}  // namespace dyx::bridge
