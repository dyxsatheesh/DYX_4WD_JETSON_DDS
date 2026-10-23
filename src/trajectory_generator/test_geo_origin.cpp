#include "geo_csv.hpp"
#include "px4_origin_guard.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
  using namespace dyx::trajectory;
  using Clock = std::chrono::steady_clock;

  const GeoPoint origin{12.9165, 79.1325};

  // Origin must project to approximately zero.
  const auto zero = project_geo(origin, origin);
  assert(std::abs(zero.x) < 0.001);
  assert(std::abs(zero.y) < 0.001);

  // Increasing longitude moves east.
  const auto east = project_geo(
      origin, {12.9165, 79.1326});
  assert(east.x > 0.0);
  assert(std::abs(east.y) < 0.1);

  // Increasing latitude moves north.
  const auto north = project_geo(
      origin, {12.9166, 79.1325});
  assert(north.y > 0.0);
  assert(std::abs(north.x) < 0.1);

  // Invalid geographic coordinates must fail.
  bool rejected = false;
  try {
    (void)project_geo(origin, {91.0, 79.0});
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  assert(rejected);

  // A PX4 origin must not become valid without evidence.
  Px4OriginGuard guard;
  assert(!guard.good(Clock::now()));

  Px4OriginSample sample{};
  sample.observed = true;
  sample.xy_global = true;
  sample.xy_valid = true;
  sample.latitude_deg = origin.latitude_deg;
  sample.longitude_deg = origin.longitude_deg;
  sample.ref_timestamp_us = 1000;
  sample.sample_timestamp_us = 2000;
  sample.received = Clock::now();

  guard.observe(sample);
  assert(guard.good(Clock::now()));

  // A changed PX4 reference must permanently invalidate it.
  sample.ref_timestamp_us = 1001;
  sample.sample_timestamp_us = 3000;
  sample.received = Clock::now();

  guard.observe(sample);
  assert(!guard.good(Clock::now()));
  assert(guard.faulted());

  std::cout
      << "PASS: Geographic projection and PX4 origin safety\n";
  return 0;
}
