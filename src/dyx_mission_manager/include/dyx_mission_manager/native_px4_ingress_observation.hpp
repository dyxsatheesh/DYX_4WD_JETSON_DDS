#pragma once
// Batch136: read-only ROS receive diagnostics. NEVER grants safety authority.
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <nlohmann/json.hpp>

namespace dyx::http_backend {
class NativePx4IngressObservation final {
 public:
  enum Topic : unsigned { vehicle, battery, position, rover_rate, gps, count };
  static constexpr double kRecentWindowSec = 1.5;
  void observe_vehicle(double now) noexcept { observe(vehicle, now); }
  void observe_battery(double now) noexcept { observe(battery, now); }
  void observe_position(double now) noexcept { observe(position, now); }
  void observe_rover_rate(double now) noexcept { observe(rover_rate, now); }
  void observe_gps(double now) noexcept { observe(gps, now); }
  nlohmann::json snapshot(double now) const {
    auto out = nlohmann::json::object();
    for (unsigned i = 0; i < count; ++i) {
      const char* names[] = {"vehicle", "battery", "position", "rover_rate", "gps"};
      const bool seen = counts_[i] > 0;
      const double age = now - received_[i];
      const bool recent = seen && std::isfinite(now) && std::isfinite(age) &&
          age >= 0.0 && age <= kRecentWindowSec;
      out[names[i]] = {
          {"received", seen}, {"messages_observed", counts_[i]},
          {"recent_transport", recent},
          {"last_receive_age_sec", seen && std::isfinite(age) && age >= 0.0
              ? nlohmann::json(age) : nlohmann::json(nullptr)}
      };
    }
    out["diagnostic_only"] = true;
    out["authorizes_motion"] = false;
    return out;
  }
 private:
  void observe(Topic topic, double now) noexcept {
    if (!std::isfinite(now) || now < 0.0) return;
    if (counts_[topic] < std::numeric_limits<std::uint64_t>::max()) ++counts_[topic];
    received_[topic] = now;
  }
  std::array<std::uint64_t, count> counts_{};
  std::array<double, count> received_{{-1.0,-1.0,-1.0,-1.0,-1.0}};
};
} // namespace dyx::http_backend
