#pragma once

#include <chrono>
#include <cmath>

namespace dyx::bridge {

using SteadyClock = std::chrono::steady_clock;

inline bool safety_evidence_fresh(
    bool received,
    SteadyClock::time_point last,
    SteadyClock::time_point now,
    double timeout_sec) {

  if (!received ||
      !std::isfinite(timeout_sec) ||
      timeout_sec <= 0.0 ||
      timeout_sec > 0.5)
    return false;

  const double age =
      std::chrono::duration<double>(now - last).count();

  return std::isfinite(age) &&
         age >= 0.0 &&
         age <= timeout_sec;
}

}  // namespace dyx::bridge
