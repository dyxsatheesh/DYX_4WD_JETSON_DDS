#pragma once

// Pure C++17 RPP primitives. Reference implementations:
// rpp_controller/path_geometry.py: wrap_angle, project_onto_segment
// rpp_controller/guidance.py: adaptive_lookahead_m, limit_bearing_to_moving_cone
// rpp_controller/speed_regulator.py: braking_distance, allowable_speed_for_distance
// DO NOT use this module to command the rover until the full RPP migration is validated.

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace dyx::rpp {

constexpr double pi = 3.141592653589793238462643383279502884;

inline void require_finite(double x, const char *name) {
  if (!std::isfinite(x)) throw std::invalid_argument(name);
}

inline void require_nonnegative(double x, const char *name) {
  require_finite(x, name);
  if (x < 0.0) throw std::invalid_argument(name);
}

struct Point2D { double x{0.0}; double y{0.0}; };
struct Segment {
  int index{0};
  Point2D start{};
  Point2D end{};
  double s_start{0.0};
};
struct Projection {
  int segment_index{0};
  double t{0.0};
  Point2D foot{};
  double signed_cross_track_m{0.0};
  double distance_m{0.0};
  double s{0.0};
};
struct LookaheadConfig {
  double lookahead_min_m{0.20};
  double lookahead_max_m{1.00};
  double lookahead_time_s{0.55};
  double xtrack_lookahead_gain{0.0};
};

inline double wrap_angle(double angle_rad) {
  require_finite(angle_rad, "angle_rad must be finite");
  double remainder = std::fmod(angle_rad + pi, 2.0 * pi);
  if (remainder < 0.0) remainder += 2.0 * pi;
  const double result = remainder - pi;
  return (result == -pi && angle_rad > 0.0) ? pi : result;
}

inline Projection project_onto_segment(Point2D query, const Segment &seg) {
  for (const double v : {query.x, query.y, seg.start.x, seg.start.y,
                         seg.end.x, seg.end.y, seg.s_start}) {
    require_finite(v, "path projection coordinates must be finite");
  }
  const double dx = seg.end.x - seg.start.x;
  const double dy = seg.end.y - seg.start.y;
  const double length_sq = dx * dx + dy * dy;
  if (!(length_sq > 0.0) || !std::isfinite(length_sq)) {
    throw std::invalid_argument("zero-length/nonfinite geometry segment");
  }
  const double rel_x = query.x - seg.start.x;
  const double rel_y = query.y - seg.start.y;
  const double t = std::clamp((rel_x * dx + rel_y * dy) / length_sq, 0.0, 1.0);
  const Point2D foot{seg.start.x + t * dx, seg.start.y + t * dy};
  const double distance = std::hypot(query.x - foot.x, query.y - foot.y);
  const double cross = dx * rel_y - dy * rel_x;
  const double signed_distance = (cross > 0.0) ? distance :
                                 (cross < 0.0) ? -distance : 0.0;
  return {seg.index, t, foot, signed_distance, distance,
          seg.s_start + t * std::sqrt(length_sq)};
}

inline double adaptive_lookahead_m(const LookaheadConfig &cfg,
                                   double speed_mps, double signed_cross_track_m) {
  require_nonnegative(cfg.lookahead_min_m, "lookahead_min_m");
  require_finite(cfg.lookahead_max_m, "lookahead_max_m");
  require_nonnegative(cfg.lookahead_time_s, "lookahead_time_s");
  require_nonnegative(cfg.xtrack_lookahead_gain, "xtrack_lookahead_gain");
  if (cfg.lookahead_max_m < cfg.lookahead_min_m) {
    throw std::invalid_argument("lookahead_max_m < lookahead_min_m");
  }
  require_nonnegative(speed_mps, "speed_mps");
  require_finite(signed_cross_track_m, "signed_cross_track_m");
  const double requested = cfg.lookahead_time_s * speed_mps +
                            cfg.xtrack_lookahead_gain * std::abs(signed_cross_track_m);
  return std::max(cfg.lookahead_min_m, std::min(cfg.lookahead_max_m, requested));
}

inline double wrap_heading_error(double target_bearing_rad, double rover_yaw_rad) {
  require_finite(target_bearing_rad, "target_bearing_rad");
  require_finite(rover_yaw_rad, "rover_yaw_rad");
  return wrap_angle(target_bearing_rad - rover_yaw_rad);
}

inline std::pair<double, bool> limit_bearing_to_moving_cone(
    double desired_bearing_rad, double rover_yaw_rad, double cone_half_angle_rad) {
  require_finite(desired_bearing_rad, "desired_bearing_rad");
  require_finite(rover_yaw_rad, "rover_yaw_rad");
  require_finite(cone_half_angle_rad, "cone_half_angle_rad");
  if (!(cone_half_angle_rad > 0.0 && cone_half_angle_rad <= pi)) {
    throw std::invalid_argument("cone_half_angle_rad must be in (0, pi]");
  }
  const double error = wrap_heading_error(desired_bearing_rad, rover_yaw_rad);
  const double limited = std::clamp(error, -cone_half_angle_rad, cone_half_angle_rad);
  const bool clamped = std::abs(limited - error) > 1.0e-15;
  return {wrap_angle(rover_yaw_rad + limited), clamped};
}

inline double braking_distance(double effective_speed_mps, double target_speed_mps,
                               double deceleration_mps2, double latency_sec,
                               double margin_m) {
  require_nonnegative(effective_speed_mps, "effective_speed_mps");
  require_nonnegative(target_speed_mps, "target_speed_mps");
  require_nonnegative(deceleration_mps2, "deceleration_mps2");
  require_nonnegative(latency_sec, "latency_sec");
  require_nonnegative(margin_m, "margin_m");
  if (deceleration_mps2 <= 0.0) throw std::invalid_argument("deceleration_mps2");
  const double distance = std::max(0.0, effective_speed_mps * effective_speed_mps -
                                       target_speed_mps * target_speed_mps) / (2.0 * deceleration_mps2);
  return distance + latency_sec * effective_speed_mps + margin_m;
}

inline double allowable_speed_for_distance(double remaining_distance_m,
                                           double target_speed_mps,
                                           double deceleration_mps2,
                                           double latency_sec,
                                           double margin_m) {
  require_nonnegative(remaining_distance_m, "remaining_distance_m");
  require_nonnegative(target_speed_mps, "target_speed_mps");
  require_nonnegative(deceleration_mps2, "deceleration_mps2");
  require_nonnegative(latency_sec, "latency_sec");
  require_nonnegative(margin_m, "margin_m");
  if (deceleration_mps2 <= 0.0) throw std::invalid_argument("deceleration_mps2");
  if (remaining_distance_m <= margin_m) return target_speed_mps;
  const double usable = remaining_distance_m - margin_m;
  const double root = std::pow(deceleration_mps2 * latency_sec, 2) +
                      target_speed_mps * target_speed_mps +
                      2.0 * deceleration_mps2 * usable;
  return std::max(target_speed_mps, -deceleration_mps2 * latency_sec + std::sqrt(root));
}

} // namespace dyx::rpp
