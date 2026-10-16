#include "dyx_rpp_cpp17_core/rear_marking_reference.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace dyx::rpp;

static bool near(double a, double b) {
  return std::abs(a - b) < 1e-8;
}

int main() {
  constexpr double pi = 3.14159265358979323846;

  // Test geometry only. These are not measured rover offsets.
  const RearMarkingLeverArm rear{-0.30, 0.0};

  auto p = compute_rear_marking_reference(10, 20, 0, rear);
  assert(near(p.north_m, 9.7));
  assert(near(p.east_m, 20));

  p = compute_rear_marking_reference(10, 20, pi / 2, rear);
  assert(near(p.north_m, 10));
  assert(near(p.east_m, 19.7));

  p = compute_rear_marking_reference(10, 20, -pi / 2, rear);
  assert(near(p.north_m, 10));
  assert(near(p.east_m, 20.3));

  p = compute_rear_marking_reference(10, 20, pi, rear);
  assert(near(p.north_m, 10.3));
  assert(near(p.east_m, 20));

  p = compute_rear_marking_reference(
      10, 20, 0, RearMarkingLeverArm{0, 0.1});
  assert(near(p.north_m, 10));
  assert(near(p.east_m, 20.1));

  bool rejected = false;
  try {
    compute_rear_marking_reference(
        0, 0, 0, RearMarkingLeverArm{3.0, 0});
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  assert(rejected);

  rejected = false;
  try {
    compute_rear_marking_reference(
        0, 0, std::numeric_limits<double>::quiet_NaN(), rear);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  assert(rejected);

  std::cout << "PASS: Rear marking-reference geometry tests\n";
}
