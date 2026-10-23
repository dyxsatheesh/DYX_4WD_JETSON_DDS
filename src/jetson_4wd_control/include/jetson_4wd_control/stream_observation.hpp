#pragma once
// Native C++17 diagnostic equivalent of Python bridge timing counters.
// Observations NEVER authorize motor commands.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace dyx::control {
struct StreamObservation {
  std::uint64_t received{0};
  std::uint64_t gaps{0};
  std::uint64_t estimated_missing{0};
  double max_gap_s{0.0};
  double last_received_s{-1.0};

  void observe(double now_s, double nominal_period_s = 0.02) {
    if (!std::isfinite(now_s) || now_s < 0.0 ||
        !std::isfinite(nominal_period_s) || nominal_period_s <= 0.0)
      return;
    if (last_received_s >= 0.0 && now_s <= last_received_s) return;
    if (last_received_s >= 0.0) {
      const double gap = now_s - last_received_s;
      max_gap_s = std::max(max_gap_s, gap);
      if (gap > 1.5 * nominal_period_s) {
        ++gaps;
        const double missed = std::round(gap / nominal_period_s) - 1.0;
        if (missed > 0.0 && missed < 1000000.0)
          estimated_missing += static_cast<std::uint64_t>(missed);
      }
    }
    ++received;
    last_received_s = now_s;
  }
  bool fresh(double now_s, double max_age_s) const {
    return last_received_s >= 0.0 && std::isfinite(now_s) &&
           std::isfinite(max_age_s) && max_age_s > 0.0 &&
           now_s >= last_received_s && now_s - last_received_s <= max_age_s;
  }
};
}  // namespace dyx::control
