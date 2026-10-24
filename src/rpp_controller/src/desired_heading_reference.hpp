#pragma once
// Desired guidance reference only; grants no physical motion authority.
#include <cmath>
#include <stdexcept>
namespace dyx::rpp {
struct HeadingReference { double east_mps, north_mps, yaw_enu_rad; };
inline HeadingReference desired_heading_reference(double forward_mps, double yaw_enu_rad) {
  if (!std::isfinite(forward_mps) || !std::isfinite(yaw_enu_rad) ||
      forward_mps < 0 || forward_mps > 1.0) throw std::invalid_argument("invalid reference");
  return {forward_mps*std::cos(yaw_enu_rad), forward_mps*std::sin(yaw_enu_rad), yaw_enu_rad};
}
}
