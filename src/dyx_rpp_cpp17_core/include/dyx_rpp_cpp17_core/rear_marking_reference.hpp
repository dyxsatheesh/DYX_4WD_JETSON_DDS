#pragma once

#include <cmath>
#include <stdexcept>

namespace dyx::rpp {

struct RearMarkingLeverArm {
  // PX4 position reference -> rear master antenna.
  // Body FRD: forward positive, right positive.
  double forward_m{0.0};
  double right_m{0.0};
};

struct RearMarkingPosition {
  double north_m{0.0};
  double east_m{0.0};
};

inline RearMarkingPosition compute_rear_marking_reference(
    double px4_north_m,
    double px4_east_m,
    double heading_ned_rad,
    RearMarkingLeverArm lever) {

  for (const double v : {
         px4_north_m, px4_east_m, heading_ned_rad,
         lever.forward_m, lever.right_m}) {
    if (!std::isfinite(v))
      throw std::invalid_argument("nonfinite marking geometry");
  }

  if (std::abs(heading_ned_rad) > 3.142 ||
      std::hypot(lever.forward_m, lever.right_m) > 2.0)
    throw std::invalid_argument("invalid marking geometry");

  const double c = std::cos(heading_ned_rad);
  const double s = std::sin(heading_ned_rad);

  RearMarkingPosition out;
  out.north_m =
      px4_north_m + lever.forward_m * c - lever.right_m * s;
  out.east_m =
      px4_east_m + lever.forward_m * s + lever.right_m * c;

  return out;
}

}  // namespace dyx::rpp
