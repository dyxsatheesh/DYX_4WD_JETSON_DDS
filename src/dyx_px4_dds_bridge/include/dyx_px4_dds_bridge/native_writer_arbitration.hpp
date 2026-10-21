#pragma once
// D3D: non-actuating ROS graph writer-count arbiter. Discovery is advisory:
// counts are not cryptographic ownership proofs, and cannot permit motion.
#include <cstddef>
#include <cstdint>
#include <initializer_list>

namespace dyx::bridge {
struct WriterCounts {
  std::size_t offboard_mode{0};
  std::size_t legacy_trajectory{0};
  std::size_t rover_speed{0};
  std::size_t rover_rate{0};
  std::size_t rover_attitude{0};
};
struct WriterArbitrationResult {
  bool graph_quiet{false};
  bool publisher_exclusivity_proven{false};
  bool motion_authorized{false};
  const char* reason{"DISCOVERY_NOT_CHECKED"};
  std::size_t observed_writers{0};
};
inline WriterArbitrationResult inspect_writer_graph(
    const WriterCounts& count, bool discovery_available,
    bool local_legacy_writer_objects_present) noexcept {
  WriterArbitrationResult r{};
  if (!discovery_available) {
    r.reason = "DISCOVERY_UNAVAILABLE";
    return r;
  }
  // Saturating summation guards against absurd counts or integer overflow.
  for (std::size_t n : {count.offboard_mode, count.legacy_trajectory,
                         count.rover_speed, count.rover_rate, count.rover_attitude}) {
    const auto limit = static_cast<std::size_t>(-1);
    r.observed_writers = n > limit - r.observed_writers ? limit : r.observed_writers + n;
  }
  if (r.observed_writers) {
    r.reason = "OTHER_OR_LEGACY_WRITER_DISCOVERED";
    return r;
  }
  if (local_legacy_writer_objects_present) {
    r.reason = "LOCAL_LEGACY_WRITER_EXISTS";
    return r;
  }
  r.graph_quiet = true;
  r.reason = "GRAPH_QUIET_NOT_EXCLUSIVE_PROOF";
  // ROS graph enumeration can race with writer creation/discovery or span
  // separate DDS domains. Never treat absence as an authority token.
  return r;
}
} // namespace dyx::bridge
