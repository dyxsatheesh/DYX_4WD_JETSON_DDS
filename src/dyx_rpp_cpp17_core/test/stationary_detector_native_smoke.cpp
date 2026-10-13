#include "dyx_rpp_cpp17_core/stationary_detector.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

static void check(bool ok) {
  if (!ok) throw std::runtime_error("stationary detector native smoke failed");
}
int main() {
  using namespace dyx::rpp;
  PositionWindowStationaryDetector d(0.5, 0.005, 0.05, 0.2);
  for (int i = 0; i <= 6; ++i) {
    auto e = d.update(i * 0.1, 0.001, 0.0, 0.02);
    if (i <= 4) check(!e.stationary);
    if (i == 5) check(e.stationary);
  }
  auto e = d.update(6 * 0.1, 0.001, 0.0, 0.02);
  check(e.stationary);
  e = d.update(0.7, 0.020, 0.0, 0.02);
  check(!e.stationary && e.sample_count == 1);
  e = d.update(0.8, 0.020, 0.0, 0.1);
  check(e.sample_count == 0 && !e.stationary);
  e = d.update(1.2, 0.020, 0.0, 0.0);
  check(e.sample_count == 1);
  e = d.update(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0);
  check(e.sample_count == 0);
  e = d.update(1.0, 0.0, 0.0, 0.0);
  bool rejected = false;
  try { d.update(0.9, 0.0, 0.0, 0.0); }
  catch (const std::invalid_argument &) { rejected = true; }
  check(rejected);
  return 0;
}
