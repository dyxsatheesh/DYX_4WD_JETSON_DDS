#include "dyx_px4_dds_bridge/geometry.hpp"
#include <cassert>
#include <cmath>
#include <limits>

int main() {
  const auto forward = dyx::convert_rpp(0.6, 0.0, 0.1, true, 1.0, 0.18);
  assert(forward.valid && std::abs(forward.north_mps - 0.6f) < 1e-6);
  assert(std::abs(forward.yaw_rate_ned_radps + 0.1f) < 1e-6);

  const auto capped = dyx::convert_rpp(1.2, 1.6, -0.5, true, 1.0, 0.18);
  assert(capped.valid);
  assert(std::abs(std::hypot(capped.north_mps, capped.east_mps) - 1.0) < 1e-6);
  assert(std::abs(capped.yaw_rate_ned_radps - 0.18f) < 1e-6);

  const auto bad = dyx::convert_rpp(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.1,
      true, 1.0, 0.18);
  assert(!bad.valid);
  assert(!dyx::convert_rpp(0.2, 0.0, 0.1, false, 1.0, 0.18).valid);
}
