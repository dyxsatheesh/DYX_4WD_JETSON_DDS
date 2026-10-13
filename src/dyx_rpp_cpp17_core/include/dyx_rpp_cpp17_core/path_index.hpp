#pragma once
// Faithful pure-C++17 port of path_geometry.py index, span, projection and
// GeometryProgressTracker. No ROS/PX4 actuation; local x=East, y=North (ENU).
#include "dyx_rpp_cpp17_core/primitives.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace dyx::rpp {

constexpr int point_pass_through = 0;
constexpr int point_dummy_alignment = 1;
constexpr int point_marking = 2;

struct RawPathPoint {
  Point2D point{};
  int point_type{point_pass_through};
  int marking_index{-1};
};
struct GeometrySegment {
  int segment_index{};
  int raw_start_index{};
  int raw_end_index{};
  Point2D start{}, end{};
  double length{}, s_start{}, s_end{}, heading_rad{};
  int leg_id{};
};
struct PathCorner {
  int corner_index{}, raw_index{};
  Point2D point{};
  double s{};
  int incoming_segment_index{}, outgoing_segment_index{};
  double incoming_heading_rad{}, outgoing_heading_rad{}, turn_angle_rad{};
  int outgoing_leg_id{};
};
struct SemanticAnchor {
  int raw_index{}, point_type{}, marking_index{};
  Point2D point{};
  std::string identity{}; // empty for dummy; nonempty for a marking point
  double s{};
  std::optional<double> incoming_heading_rad, outgoing_heading_rad;
};
struct ActiveSemanticSpan {
  int start_raw_index{}, stop_raw_index{};
  double start_s{}, stop_s{};
  std::optional<int> first_segment_index, last_segment_index;
  std::string active_goal_identity{};
};
struct ArcLengthTarget {
  std::optional<int> segment_index;
  double t{};
  Point2D point{};
  double s{};
  std::optional<double> heading_rad;
};
struct PathProjection {
  std::optional<int> segment_index;
  int raw_start_index{}, raw_end_index{};
  double t{};
  Point2D point{};
  double signed_cross_track_m{}, distance_m{};
  double projected_s{}, progress_s{};
  bool monotonic_clamped{};
  double remaining_to_active_stop_m{}, remaining_path_m{};
  std::optional<double> next_corner_distance_m, next_corner_angle_rad,
      next_leg_heading_rad;
  bool used_full_reacquire{};
};
struct ProjectOptions {
  std::optional<int> hint_segment_index{};
  int back_window_segments{2};
  int forward_window_segments{4};
  std::optional<double> reference_s{};
  double max_backward_jump_m{0.10};
  std::optional<double> max_forward_jump_m{};
  std::optional<double> full_reacquire_distance_m{};
};

class PathGeometryIndex {
public:
  static PathGeometryIndex build(const std::vector<RawPathPoint> &source,
                                  double corner_threshold_rad = pi / 4.0,
                                  double zero_length_epsilon_m = 1e-12) {
    if (source.empty()) throw std::invalid_argument("raw path empty");
    require_finite(corner_threshold_rad, "corner_threshold_rad");
    require_nonnegative(zero_length_epsilon_m, "zero_length_epsilon_m");
    if (corner_threshold_rad < 0 || corner_threshold_rad > pi)
      throw std::invalid_argument("corner_threshold_rad outside [0,pi]");
    PathGeometryIndex result;
    result.raw_points = source;
    int markers = 0;
    for (const auto &p : source) {
      require_finite(p.point.x, "raw x");
      require_finite(p.point.y, "raw y");
      if (p.point_type < 0 || p.point_type > 2) throw std::invalid_argument("unknown point_type");
      if (p.point_type == point_marking) {
        if (p.marking_index != markers++) throw std::invalid_argument("marking indices not consecutive");
      } else if (p.marking_index != -1) {
        throw std::invalid_argument("non-marking index must equal -1");
      }
    }
    result.corner_threshold_rad = corner_threshold_rad;
    result.raw_s_by_index.reserve(source.size());
    result.raw_s_by_index.push_back(0.0);
    double cumulative = 0.0;
    for (size_t i = 1; i < source.size(); ++i) {
      const auto &a = source[i-1].point;
      const auto &b = source[i].point;
      const double length = std::hypot(b.x-a.x, b.y-a.y);
      require_finite(length, "segment length overflow");
      cumulative += length;
      require_finite(cumulative, "path length overflow");
      result.raw_s_by_index.push_back(cumulative);
    }
    result.total_length = cumulative;
    result.raw_segment_to_geometry.resize(source.size()-1);
    for (size_t i = 0; i+1 < source.size(); ++i) {
      const double length = result.raw_s_by_index[i+1]-result.raw_s_by_index[i];
      if (length <= zero_length_epsilon_m) continue;
      const Point2D a = source[i].point;
      const Point2D b = source[i+1].point;
      int index = static_cast<int>(result.segments.size());
      result.segments.push_back({index, static_cast<int>(i), static_cast<int>(i+1),
                                 a, b, length, result.raw_s_by_index[i],
                                 result.raw_s_by_index[i+1],
                                 std::atan2(b.y-a.y, b.x-a.x), 0});
      result.raw_segment_to_geometry[i] = index;
    }
    int leg = 0;
    for (size_t i = 0; i < result.segments.size(); ++i) {
      if (i > 0) {
        const auto &incoming = result.segments[i-1];
        const auto &outgoing = result.segments[i];
        const double turn = wrap_angle(outgoing.heading_rad-incoming.heading_rad);
        if (std::abs(turn) > 1e-15 && std::abs(turn)+1e-15 >= corner_threshold_rad) {
          ++leg;
          int raw = outgoing.raw_start_index;
          for (int r=incoming.raw_end_index; r<=outgoing.raw_start_index; ++r) {
            if (source.at(static_cast<size_t>(r)).point_type != point_pass_through) {
              raw=r; break;
            }
          }
          result.corners.push_back({static_cast<int>(result.corners.size()), raw,
            source[static_cast<size_t>(raw)].point, result.raw_s_by_index[static_cast<size_t>(raw)],
            incoming.segment_index, outgoing.segment_index, incoming.heading_rad,
            outgoing.heading_rad, turn, leg});
        }
      }
      result.segments[i].leg_id=leg;
    }
    for (size_t i = 0; i < source.size(); ++i) {
      const auto &p=source[i];
      if (p.point_type == point_pass_through) continue;
      SemanticAnchor a;
      a.raw_index=static_cast<int>(i);
      a.point_type=p.point_type;
      a.marking_index=p.marking_index;
      a.point=p.point;
      a.s=result.raw_s_by_index[i];
      if (p.point_type == point_marking) {
        char buffer[48]{};
        std::snprintf(buffer, sizeof(buffer), "P%04d", p.marking_index+1);
        a.identity=buffer;
      }
      for (const auto &s: result.segments) {
        if (s.raw_end_index<=static_cast<int>(i)) a.incoming_heading_rad=s.heading_rad;
        if (!a.outgoing_heading_rad && s.raw_start_index>=static_cast<int>(i))
          a.outgoing_heading_rad=s.heading_rad;
      }
      result.semantic_anchors.push_back(a);
    }
    return result;
  }

  std::vector<RawPathPoint> raw_points;
  std::vector<double> raw_s_by_index;
  std::vector<std::optional<int>> raw_segment_to_geometry;
  std::vector<GeometrySegment> segments;
  std::vector<SemanticAnchor> semantic_anchors;
  std::vector<PathCorner> corners;
  double total_length{};
  double corner_threshold_rad{};

  ActiveSemanticSpan active_span(int stop_raw_index,
                                  std::optional<int> start_raw_index = {}) const {
    validate_index(stop_raw_index);
    if (!start_raw_index) {
      int previous=0;
      for (const auto &a:semantic_anchors)
        if (a.raw_index<stop_raw_index) previous=a.raw_index;
      start_raw_index=previous;
    }
    validate_index(*start_raw_index);
    if (*start_raw_index>stop_raw_index) throw std::invalid_argument("span start > stop");
    const SemanticAnchor *goal=semantic_anchor_at(stop_raw_index);
    if (!goal) throw std::invalid_argument("span stop is not semantic anchor");
    const auto [first,last]=segment_bounds(*start_raw_index,stop_raw_index);
    return {*start_raw_index,stop_raw_index,
      raw_s_by_index[static_cast<size_t>(*start_raw_index)],
      raw_s_by_index[static_cast<size_t>(stop_raw_index)],first,last,goal->identity};
  }

  const SemanticAnchor *semantic_anchor_at(int index) const {
    validate_index(index);
    for (const auto &a:semantic_anchors) if (a.raw_index==index) return &a;
    return nullptr;
  }

  const PathCorner *next_corner(double s) const {
    require_finite(s,"corner s");
    for (const auto &c:corners) if (c.s>s) return &c;
    return nullptr;
  }

  PathProjection project(Point2D query, const ActiveSemanticSpan &span,
                         const ProjectOptions &opt = {}) const {
    require_finite(query.x,"query.x");
    require_finite(query.y,"query.y");
    validate_span(span);
    if (opt.back_window_segments<0 || opt.forward_window_segments<0)
      throw std::invalid_argument("negative segment window");
    if (opt.reference_s) require_finite(*opt.reference_s,"reference_s");
    require_nonnegative(opt.max_backward_jump_m,"max_backward_jump_m");
    if (opt.max_forward_jump_m) require_nonnegative(*opt.max_forward_jump_m,"max_forward_jump_m");
    if (opt.full_reacquire_distance_m)
      require_nonnegative(*opt.full_reacquire_distance_m,"full_reacquire_distance_m");
    if (!span.first_segment_index) {
      const auto &anchor=raw_points[static_cast<size_t>(span.stop_raw_index)];
      return enrich({},span.stop_raw_index,span.stop_raw_index,anchor.point,
        std::hypot(query.x-anchor.point.x,query.y-anchor.point.y),span.stop_s,span,false);
    }
    const int first=*span.first_segment_index, last=*span.last_segment_index;
    int lo=first,hi=last;
    if (opt.hint_segment_index && *opt.hint_segment_index>=first && *opt.hint_segment_index<=last) {
      lo=std::max(first,*opt.hint_segment_index-opt.back_window_segments);
      hi=std::min(last,*opt.hint_segment_index+opt.forward_window_segments);
    }
    const bool partial=(lo!=first || hi!=last);
    std::optional<Projection> best=best_projection(query,lo,hi,opt);
    bool full=false;
    if (opt.full_reacquire_distance_m && partial &&
        (!best || best->distance_m>*opt.full_reacquire_distance_m)) {
      best=best_projection(query,first,last,opt);
      full=true;
    }
    if (!best) throw std::invalid_argument("jump bounds exclude every projection candidate");
    const auto &s=segments.at(static_cast<size_t>(best->segment_index));
    return enrich(best,s.raw_start_index,s.raw_end_index,best->foot,best->distance_m,best->s,span,full);
  }

  ArcLengthTarget point_at_s(double s, const std::optional<ActiveSemanticSpan> &span={}) const {
    require_finite(s,"point_at_s s");
    if (span) validate_span(*span);
    const double lo=span?span->start_s:0.0;
    const double hi=span?span->stop_s:total_length;
    const double target=std::clamp(s,lo,hi);
    if (segments.empty() || lo==hi) {
      const int raw=span?span->stop_raw_index:(target<=0.0?0:static_cast<int>(raw_points.size()-1));
      return { {},0.0,raw_points[static_cast<size_t>(raw)].point,target,{} };
    }
    int first=0,last=static_cast<int>(segments.size()-1);
    if (span && span->first_segment_index) {
      first=*span->first_segment_index;
      last=*span->last_segment_index;
    }
    if (first>last) {
      return {{},0.0,raw_points[static_cast<size_t>(span->stop_raw_index)].point,target,{}};
    }
    int chosen=last;
    for (int i=first;i<=last;++i) {
      if (segments[static_cast<size_t>(i)].s_end >= target-1e-12) {chosen=i;break;}
    }
    const auto &p=segments[static_cast<size_t>(chosen)];
    const double t=std::clamp((target-p.s_start)/p.length,0.0,1.0);
    return {p.segment_index,t,{p.start.x+t*(p.end.x-p.start.x),
      p.start.y+t*(p.end.y-p.start.y)},target,p.heading_rad};
  }

  ArcLengthTarget lookahead_target(double progress_s,double lookahead_m,
                             const std::optional<ActiveSemanticSpan> &span={}) const {
    require_nonnegative(lookahead_m,"lookahead_m");
    return point_at_s(progress_s+lookahead_m,span);
  }

private:
  PathGeometryIndex()=default;
  void validate_index(int idx) const {
    if (idx<0 || static_cast<size_t>(idx)>=raw_points.size())
      throw std::invalid_argument("raw index outside path");
  }
  std::pair<std::optional<int>,std::optional<int>> segment_bounds(int start,int stop) const {
    int first=0;
    while (first<static_cast<int>(segments.size()) && segments[static_cast<size_t>(first)].raw_start_index<start) ++first;
    int last=static_cast<int>(segments.size())-1;
    while (last>=0 && segments[static_cast<size_t>(last)].raw_end_index>stop) --last;
    if (first>=static_cast<int>(segments.size()) || first>last ||
        segments[static_cast<size_t>(first)].raw_end_index>stop) return {{},{}};
    return {first,last};
  }
  void validate_span(const ActiveSemanticSpan &s) const {
    validate_index(s.start_raw_index);
    validate_index(s.stop_raw_index);
    if (s.start_raw_index>s.stop_raw_index) throw std::invalid_argument("bad active span");
    if (s.start_s!=raw_s_by_index[static_cast<size_t>(s.start_raw_index)] ||
        s.stop_s!=raw_s_by_index[static_cast<size_t>(s.stop_raw_index)])
      throw std::invalid_argument("active span not from this geometry");
    const auto *goal=semantic_anchor_at(s.stop_raw_index);
    if (!goal || goal->identity!=s.active_goal_identity)
      throw std::invalid_argument("active span goal invalid");
    const auto [first,last]=segment_bounds(s.start_raw_index,s.stop_raw_index);
    if (first!=s.first_segment_index || last!=s.last_segment_index)
      throw std::invalid_argument("active span geometry mismatch");
  }
  std::optional<Projection> best_projection(Point2D query,int first,int last,
                                             const ProjectOptions &opt) const {
    std::optional<Projection> best;
    double reference=0.0;
    bool have_reference=false;
    if (opt.reference_s) {reference=*opt.reference_s;have_reference=true;}
    for (int i=first;i<=last;++i) {
      const auto &s=segments.at(static_cast<size_t>(i));
      Projection p=project_onto_segment(query,{s.segment_index,s.start,s.end,s.s_start});
      if (opt.reference_s && p.s<*opt.reference_s-opt.max_backward_jump_m) continue;
      if (opt.reference_s && opt.max_forward_jump_m && p.s>*opt.reference_s+*opt.max_forward_jump_m) continue;
      if (!have_reference) {reference=p.s;have_reference=true;}
      if (!best || std::tuple<double,double,int>(p.distance_m,std::abs(p.s-reference),p.segment_index)<
                   std::tuple<double,double,int>(best->distance_m,std::abs(best->s-reference),best->segment_index)) best=p;
    }
    return best;
  }
  PathProjection enrich(std::optional<Projection> p,int raw_start,int raw_end,
        Point2D point,double dist,double s,const ActiveSemanticSpan &span,bool used_full) const {
    PathProjection out;
    out.segment_index=p?std::optional<int>(p->segment_index):std::nullopt;
    out.raw_start_index=raw_start;
    out.raw_end_index=raw_end;
    out.t=p?p->t:0.0;
    out.point=point;
    out.signed_cross_track_m=p?p->signed_cross_track_m:0.0;
    out.distance_m=dist;
    out.projected_s=s;
    out.progress_s=s;
    out.remaining_to_active_stop_m=std::max(0.0,span.stop_s-s);
    out.remaining_path_m=std::max(0.0,total_length-s);
    if (const auto *corner=next_corner(s)) {
      out.next_corner_distance_m=std::max(0.0,corner->s-s);
      out.next_corner_angle_rad=corner->turn_angle_rad;
      out.next_leg_heading_rad=corner->outgoing_heading_rad;
    }
    out.used_full_reacquire=used_full;
    return out;
  }
};

enum class GeometryResetReason {initial_install,path_replaced,active_goal_advanced,
                  localization_jump,source_cleared,manual};

class GeometryProgressTracker {
public:
  explicit GeometryProgressTracker(const PathGeometryIndex &geometry):geometry_(geometry) {}
  void reset(GeometryResetReason reason,std::optional<double> progress={},std::optional<int> hint={}) {
    if (progress) {
      require_finite(*progress,"reset progress");
      if (*progress<0 || *progress>geometry_.total_length) throw std::invalid_argument("reset progress outside path");
    }
    if (hint && (*hint<0 || static_cast<size_t>(*hint)>=geometry_.segments.size()))
      throw std::invalid_argument("reset hint outside path");
    progress_s=progress;
    hint_segment_index=hint;
    last_reset_reason=reason;
    ++reset_count;
  }
  PathProjection update(Point2D query,const ActiveSemanticSpan &span,ProjectOptions opt={}) {
    opt.reference_s=progress_s;
    opt.hint_segment_index=hint_segment_index;
    PathProjection out=geometry_.project(query,span,opt);
    const double old=progress_s?std::clamp(*progress_s,span.start_s,span.stop_s):span.start_s;
    const double progress=std::max(old,out.projected_s);
    out.monotonic_clamped=(progress>out.projected_s+1e-12);
    out.progress_s=progress;
    out.remaining_to_active_stop_m=std::max(0.0,span.stop_s-progress);
    out.remaining_path_m=std::max(0.0,geometry_.total_length-progress);
    out.next_corner_distance_m.reset();
    out.next_corner_angle_rad.reset();
    out.next_leg_heading_rad.reset();
    if (const auto *corner=geometry_.next_corner(progress)) {
      out.next_corner_distance_m=std::max(0.0,corner->s-progress);
      out.next_corner_angle_rad=corner->turn_angle_rad;
      out.next_leg_heading_rad=corner->outgoing_heading_rad;
    }
    progress_s=progress;
    if (out.segment_index) hint_segment_index=out.segment_index;
    return out;
  }
  std::optional<int> hint_segment_index{};
  std::optional<double> progress_s{};
  GeometryResetReason last_reset_reason{GeometryResetReason::initial_install};
  int reset_count{1};
private:
  const PathGeometryIndex &geometry_;
};
} // namespace dyx::rpp
