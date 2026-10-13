#pragma once
// Phase 2G2A: non-actuating metric trajectory compiler for a forward-only rover.
// Source-of-truth rules: trajectory_generator/topology.py and
// trajectory_generator_node.py::_generate_navigation_path (as of fa2d461).
// This class neither commands motors nor authorizes a manoeuvre/spray.

#include "dyx_rpp_cpp17_core/path_index.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dyx::rpp {

enum class RowTransitionReason {
  disabled, last_pair, not_row_end, gap_not_under_trigger,
  skipped_by_operator, added
};
inline const char *row_transition_name(RowTransitionReason v) {
  switch (v) {
    case RowTransitionReason::disabled: return "DISABLED";
    case RowTransitionReason::last_pair: return "LAST_PAIR";
    case RowTransitionReason::not_row_end: return "NOT_ROW_END";
    case RowTransitionReason::gap_not_under_trigger: return "SKIPPED_GAP";
    case RowTransitionReason::skipped_by_operator: return "SKIPPED_OPERATOR";
    case RowTransitionReason::added: return "ADDED";
  }
  return "INVALID";
}
struct TrajectoryBuildConfig {
  bool extension_enabled{false};
  double extension_trigger_m{3.0};       // strict gap < threshold
  double extension_distance_m{4.0};      // operator configurable, positive
  double interpolation_spacing_m{0.05};
  double minimum_segment_length_m{0.001};
  double minimum_dummy_clearance_m{0.05};
  double max_abs_coordinate_m{10000.0};
  std::size_t maximum_marking_points{10000};
  std::size_t maximum_navigation_points{200000};
  double row_transfer_min_angle_deg{45.0};
  double row_transfer_max_angle_deg{135.0};
  double row_reversal_min_angle_deg{135.0};
  // Zero-based row-end marking indexes. Only eligible row ends are skipped.
  std::set<std::size_t> skip_row_end_indices{};

  void validate() const {
    for (double x : {extension_trigger_m, extension_distance_m, interpolation_spacing_m,
                     minimum_segment_length_m, minimum_dummy_clearance_m,
                     max_abs_coordinate_m}) {
      if (!std::isfinite(x) || x <= 0.0) throw std::invalid_argument("invalid trajectory length setting");
    }
    for (double x : {row_transfer_min_angle_deg, row_transfer_max_angle_deg,
                     row_reversal_min_angle_deg}) {
      if (!std::isfinite(x) || x < 0.0 || x > 180.0)
        throw std::invalid_argument("invalid row transition angle");
    }
    if (row_transfer_min_angle_deg > row_transfer_max_angle_deg ||
        maximum_marking_points < 2 || maximum_navigation_points < maximum_marking_points ||
        minimum_dummy_clearance_m > extension_distance_m ||
        interpolation_spacing_m < minimum_segment_length_m)
      throw std::invalid_argument("inconsistent trajectory configuration");
  }
};
struct RowTransitionDecision {
  std::size_t from_marking_index{};
  RowTransitionReason reason{RowTransitionReason::disabled};
  double gap_m{};
  double transfer_angle_deg{std::numeric_limits<double>::quiet_NaN()};
  double reversal_angle_deg{std::numeric_limits<double>::quiet_NaN()};
  bool has_dummy{false};
  Point2D dummy{};
};
struct NativeTrajectory {
  // Local ENU x=east/y=north; marking coordinates remain exact input values.
  std::vector<RawPathPoint> navigation{};
  std::vector<std::size_t> marking_raw_indices{};
  std::vector<RowTransitionDecision> transitions{};
  std::size_t dummy_count{0};
};

class NativeTrajectoryBuilder {
 public:
  explicit NativeTrajectoryBuilder(TrajectoryBuildConfig cfg = {}) : cfg_(std::move(cfg)) {
    cfg_.validate();
  }

  NativeTrajectory build(const std::vector<Point2D> &markings) const {
    if (markings.size() < 2 || markings.size() > cfg_.maximum_marking_points)
      throw std::invalid_argument("marking count outside allowed range");
    for (const auto &p : markings) validate_point(p);
    for (std::size_t i = 1; i < markings.size(); ++i) {
      const double gap = distance(markings[i-1], markings[i]);
      if (!std::isfinite(gap) || gap < cfg_.minimum_segment_length_m)
        throw std::invalid_argument("consecutive marking points too close");
    }

    NativeTrajectory result;
    append(result, markings[0], point_marking, 0);
    result.marking_raw_indices.push_back(0);
    for (std::size_t i = 0; i + 1 < markings.size(); ++i) {
      const RowTransitionDecision decision = classify(markings, i);
      result.transitions.push_back(decision);
      if (decision.has_dummy) {
        append_segment(result, markings[i], decision.dummy, point_dummy_alignment, -1);
        ++result.dummy_count;
        append_segment(result, decision.dummy, markings[i+1], point_marking,
                       static_cast<int>(i+1));
      } else {
        append_segment(result, markings[i], markings[i+1], point_marking,
                       static_cast<int>(i+1));
      }
      result.marking_raw_indices.push_back(result.navigation.size()-1);
    }
    if (result.marking_raw_indices.size() != markings.size())
      throw std::logic_error("marking count drift");
    for (std::size_t i = 0; i < markings.size(); ++i) {
      const auto &p = result.navigation[result.marking_raw_indices[i]];
      if (p.point_type != point_marking || p.marking_index != static_cast<int>(i) ||
          p.point.x != markings[i].x || p.point.y != markings[i].y)
        throw std::logic_error("marking coordinate mutated");
    }
    return result;
  }

 private:
  void validate_point(Point2D p) const {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        std::abs(p.x) > cfg_.max_abs_coordinate_m ||
        std::abs(p.y) > cfg_.max_abs_coordinate_m)
      throw std::invalid_argument("marking outside local-coordinate bounds");
  }
  static double distance(Point2D a, Point2D b) {
    return std::hypot(b.x-a.x, b.y-a.y);
  }
  static double vector_angle_deg(Point2D a, Point2D b) {
    const double lengths = std::hypot(a.x,a.y) * std::hypot(b.x,b.y);
    if (!std::isfinite(lengths) || lengths <= 0.0)
      throw std::invalid_argument("zero/overflow direction vector");
    const double cosine = std::clamp((a.x*b.x + a.y*b.y)/lengths, -1.0, 1.0);
    return std::acos(cosine) * 180.0 / pi;
  }
  RowTransitionDecision classify(const std::vector<Point2D> &p, std::size_t i) const {
    RowTransitionDecision r;
    r.from_marking_index = i;
    r.gap_m = distance(p[i], p[i+1]);
    if (i+2 >= p.size()) { r.reason = RowTransitionReason::last_pair; return r; }
    const Point2D transfer{p[i+1].x-p[i].x, p[i+1].y-p[i].y};
    const Point2D outgoing{p[i+2].x-p[i+1].x, p[i+2].y-p[i+1].y};
    const Point2D incoming = i > 0
        ? Point2D{p[i].x-p[i-1].x, p[i].y-p[i-1].y}
        : Point2D{-outgoing.x,-outgoing.y};
    r.transfer_angle_deg = vector_angle_deg(incoming,transfer);
    r.reversal_angle_deg = vector_angle_deg(incoming,outgoing);
    const bool row_end = r.transfer_angle_deg >= cfg_.row_transfer_min_angle_deg &&
                         r.transfer_angle_deg <= cfg_.row_transfer_max_angle_deg &&
                         r.reversal_angle_deg >= cfg_.row_reversal_min_angle_deg;
    if (!row_end) { r.reason=RowTransitionReason::not_row_end; return r; }
    if (!cfg_.extension_enabled) { r.reason=RowTransitionReason::disabled; return r; }
    if (!(r.gap_m < cfg_.extension_trigger_m)) {
      r.reason=RowTransitionReason::gap_not_under_trigger; return r;
    }
    if (cfg_.skip_row_end_indices.count(i)) {
      r.reason=RowTransitionReason::skipped_by_operator; return r;
    }
    const double incoming_norm = std::hypot(incoming.x,incoming.y);
    r.dummy = {p[i].x + incoming.x / incoming_norm * cfg_.extension_distance_m,
               p[i].y + incoming.y / incoming_norm * cfg_.extension_distance_m};
    validate_point(r.dummy);
    if (distance(p[i],r.dummy) < cfg_.minimum_dummy_clearance_m)
      throw std::invalid_argument("dummy clearance too small");
    if (distance(r.dummy,p[i+1]) < cfg_.minimum_segment_length_m)
      throw std::invalid_argument("dummy overlaps following marking");
    r.reason=RowTransitionReason::added;
    r.has_dummy=true;
    return r;
  }
  void append(NativeTrajectory &t, Point2D p, int type, int index) const {
    validate_point(p);
    if (t.navigation.size() >= cfg_.maximum_navigation_points)
      throw std::length_error("navigation point limit exceeded");
    if (!t.navigation.empty() && distance(t.navigation.back().point,p) < cfg_.minimum_segment_length_m)
      throw std::invalid_argument("generated path points too close");
    t.navigation.push_back({p,type,index});
  }
  void append_segment(NativeTrajectory &t, Point2D start, Point2D end,
                      int last_type, int last_index) const {
    const double length = distance(start,end);
    if (!std::isfinite(length) || length < cfg_.minimum_segment_length_m)
      throw std::invalid_argument("zero-length interpolation segment");
    const double count = std::ceil(length/cfg_.interpolation_spacing_m);
    if (!std::isfinite(count) || count > static_cast<double>(cfg_.maximum_navigation_points - t.navigation.size()))
      throw std::length_error("navigation point limit exceeded");
    const std::size_t divisions = std::max<std::size_t>(1, static_cast<std::size_t>(count));
    const double dx = end.x-start.x, dy=end.y-start.y;
    for (std::size_t step=1; step<=divisions; ++step) {
      const bool final = step==divisions;
      const double tstep=static_cast<double>(step)/static_cast<double>(divisions);
      const Point2D point = final ? end : Point2D{start.x + tstep*dx,start.y + tstep*dy};
      append(t,point,final?last_type:point_pass_through,final?last_index:-1);
    }
  }
  TrajectoryBuildConfig cfg_{};
};
}  // namespace dyx::rpp
