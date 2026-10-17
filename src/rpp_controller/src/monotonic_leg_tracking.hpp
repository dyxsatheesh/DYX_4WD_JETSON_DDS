#pragma once
#include "dyx_rpp_cpp17_core/path_index.hpp"
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>
namespace dyx::rpp {
struct TrackedLegDiagnostic {
  double progress_m{}, remaining_m{}, cross_track_m{};
  Point2D lookahead{};
  bool monotonic_clamped{};
};
class MonotonicLegTracking final {
public:
  void reset() { tracker_.reset(); geometry_=nullptr; mark_=-1; revision_=0; }
  TrackedLegDiagnostic step(const PathGeometryIndex& geometry,
      const std::vector<std::int32_t>& marks,std::int32_t active_mark,
      std::uint64_t revision,Point2D vehicle,double lookahead_m,double max_jump_m) {
    if(active_mark<1 || static_cast<std::size_t>(active_mark)>=marks.size())
      throw std::invalid_argument("TRACKER_NEEDS_SELECTED_P2_PLUS");
    if(!std::isfinite(lookahead_m)||lookahead_m<0||lookahead_m>5.0||
       !std::isfinite(max_jump_m)||max_jump_m<=0.0||max_jump_m>5.0)
      throw std::invalid_argument("TRACKER_BAD_LIMITS");
    const auto first=marks.at(static_cast<std::size_t>(active_mark-1));
    const auto stop=marks.at(static_cast<std::size_t>(active_mark));
    if(first<0 || stop<=first || static_cast<std::size_t>(stop)>=geometry.raw_points.size() ||
       geometry.raw_points[static_cast<std::size_t>(first)].point_type!=point_marking ||
       geometry.raw_points[static_cast<std::size_t>(stop)].point_type!=point_marking ||
       geometry.raw_points[static_cast<std::size_t>(stop)].marking_index!=active_mark)
      throw std::invalid_argument("TRACKER_SEMANTICS_INVALID");
    if(geometry_!=&geometry || mark_!=active_mark || revision_!=revision) {
      reset(); geometry_=&geometry; mark_=active_mark; revision_=revision;
      tracker_=std::make_unique<GeometryProgressTracker>(geometry);
    }
    const auto span=geometry.active_span(stop,first);
    ProjectOptions opts;
    opts.max_backward_jump_m=0.10;
    // A 0.5m upper jump in one 50ms step is far faster than the rover's rated speed.
    // Apply only from the second update, not initial path acquisition.
    if(tracker_->progress_s) opts.max_forward_jump_m=max_jump_m;
    opts.full_reacquire_distance_m=std::max(0.25,max_jump_m);
    const auto projection=tracker_->update(vehicle,span,opts);
    const auto target=geometry.lookahead_target(projection.progress_s,lookahead_m,span);
    return {projection.progress_s-span.start_s,
            projection.remaining_to_active_stop_m,
            projection.signed_cross_track_m,target.point,projection.monotonic_clamped};
  }
private:
  const PathGeometryIndex* geometry_{nullptr};
  std::int32_t mark_{-1};
  std::uint64_t revision_{0};
  std::unique_ptr<GeometryProgressTracker> tracker_{};
};
} // namespace dyx::rpp
