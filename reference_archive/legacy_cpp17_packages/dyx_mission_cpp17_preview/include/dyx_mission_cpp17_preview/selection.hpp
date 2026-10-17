#pragma once
// H6C: ROS-free mission preview selection only. NEVER a mission command,
// marking certificate, spray authorization, or physical motion permission.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
namespace dyx::mission_preview {
struct NedPoint { double north_m{}; double east_m{}; };
struct Selection {
  std::vector<NedPoint> markings{};
  std::size_t selected_index{};
  NedPoint chosen{};
  bool physical_command_permitted{false};
};
inline Selection validate_selection(const std::vector<double>& flat_ned,
                                    std::int64_t index) {
  if (flat_ned.size() < 4 || (flat_ned.size() & 1U) || flat_ned.size() > 20000)
    throw std::invalid_argument("H6C requires 2..10000 NED point pairs");
  if (index < 0 || static_cast<std::size_t>(index) >= flat_ned.size() / 2)
    throw std::out_of_range("H6C selection outside waypoint count");
  Selection result;
  result.selected_index = static_cast<std::size_t>(index);
  result.markings.reserve(flat_ned.size() / 2);
  for (std::size_t i = 0; i < flat_ned.size(); i += 2) {
    const double n = flat_ned[i], e = flat_ned[i + 1];
    if (!std::isfinite(n) || !std::isfinite(e) ||
        std::abs(n) > 10000.0 || std::abs(e) > 10000.0)
      throw std::invalid_argument("H6C invalid NED coordinate");
    if (!result.markings.empty()) {
      const auto previous = result.markings.back();
      if (std::hypot(n-previous.north_m, e-previous.east_m) < 0.001)
        throw std::invalid_argument("H6C repeated adjacent marking point");
    }
    result.markings.push_back({n,e});
  }
  result.chosen = result.markings[result.selected_index];
  return result;
}
} // namespace dyx::mission_preview
