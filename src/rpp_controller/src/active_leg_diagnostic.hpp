#pragma once
#include "dyx_rpp_cpp17_core/path_index.hpp"
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
namespace dyx::rpp {
struct ActiveLegDiagnostic {
  std::string goal_identity;
  double radial_m{0},cross_track_m{0},remaining_m{0};
  Point2D lookahead{};
};
inline ActiveLegDiagnostic inspect_active_leg(
    const PathGeometryIndex& geometry,
    const std::vector<std::int32_t>& marking_raw_indices,
    std::int32_t marking_index,Point2D rover,double lookahead_m){
  if(marking_index<=0 || static_cast<std::size_t>(marking_index)>=marking_raw_indices.size())
    throw std::invalid_argument("ACTIVE_MARKING_OUTSIDE_SUPPORTED_LEG");
  if(!std::isfinite(lookahead_m)||lookahead_m<0||lookahead_m>5.0)
    throw std::invalid_argument("INVALID_LOOKAHEAD");
  const int first=marking_raw_indices.at(static_cast<std::size_t>(marking_index-1));
  const int last=marking_raw_indices.at(static_cast<std::size_t>(marking_index));
  const auto span=geometry.active_span(last,first);
  const auto projected=geometry.project(rover,span);
  const auto target=geometry.lookahead_target(projected.progress_s,lookahead_m,span);
  const auto& terminal=geometry.raw_points.at(static_cast<std::size_t>(last));
  return {span.active_goal_identity,
      std::hypot(rover.x-terminal.point.x,rover.y-terminal.point.y),
      projected.signed_cross_track_m,projected.remaining_to_active_stop_m,target.point};
}
}
