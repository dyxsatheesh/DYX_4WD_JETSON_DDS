#pragma once
#include "dyx_rpp_cpp17_core/path_index.hpp"
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace dyx::rpp {
inline std::vector<RawPathPoint> attach_semantics(
    const std::vector<RawPathPoint>& plain,
    const std::vector<std::uint8_t>& types,
    const std::vector<std::int32_t>& marks) {
  if (plain.size() < 2 || plain.size() != types.size())
    throw std::invalid_argument("PATH_TYPES_LENGTH_MISMATCH");
  if (marks.size() < 2 || marks.size() > plain.size())
    throw std::invalid_argument("MARKING_COUNT_INVALID");
  std::vector<RawPathPoint> enriched = plain;
  std::size_t found = 0;
  for (std::size_t i=0;i<plain.size();++i) {
    if (types[i] > 2) throw std::invalid_argument("UNKNOWN_PATH_POINT_TYPE");
    enriched[i].point_type = static_cast<int>(types[i]);
    enriched[i].marking_index = -1;
    if (types[i] == point_marking) {
      if (found >= marks.size() || marks[found] != static_cast<std::int32_t>(i))
        throw std::invalid_argument("MARKING_INDEX_ORDER_MISMATCH");
      enriched[i].marking_index = static_cast<int>(found++);
    }
  }
  if (found != marks.size() || marks.front() != 0 ||
      marks.back() != static_cast<std::int32_t>(plain.size()-1))
    throw std::invalid_argument("MARKING_ENDPOINTS_MISMATCH");
  // Shared geometry library validates contiguous semantic identities.
  (void)PathGeometryIndex::build(enriched);
  return enriched;
}
}
