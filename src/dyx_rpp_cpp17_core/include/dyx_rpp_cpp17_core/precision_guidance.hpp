#pragma once
// Phase 2C1: pure ENU precision guidance port. Reference:
// rpp_controller/rpp_controller/guidance.py::compute_precision_guidance.
// No ROS, actuator, or speed/yaw-rate authority.
#include "dyx_rpp_cpp17_core/primitives.hpp"
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

namespace dyx::rpp {

struct PrecisionGuidanceConfig {
  LookaheadConfig lookahead{};
  double moving_bearing_cone_rad{pi / 6.0};
};

struct PrecisionGuidanceResult {
  double lookahead_distance_m{};
  double lookahead_target_s{};
  std::optional<int> lookahead_segment_index;
  Point2D lookahead_point{}, steering_target_point{};
  double actual_steering_target_distance_m{};
  bool endpoint_extension_used{};
  double endpoint_extension_distance_m{};
  bool target_behind_rover{};
  double lookahead_bearing_rad{}, local_path_heading_rad{};
  double path_heading_error_rad{}, heading_error_rad{};
  double signed_cross_track_m{}, desired_movement_bearing_rad{};
  double limited_command_bearing_rad{}, final_command_correction_rad{};
  bool bearing_clamp_fired{}, zero_vector_fallback_used{};
};

// Template permits a ROS-free/native build; instantiated by the real
// PathGeometryIndex and PathProjection from path_index.hpp in the rover.
// The geometry is authoritative: virtual endpoint extension changes ONLY
// the steering target, NEVER the path/marking coordinates or progress.
template <class Geometry, class ProjectionT, class Span>
PrecisionGuidanceResult compute_precision_guidance(
    const PrecisionGuidanceConfig &cfg, const Geometry &geometry,
    const ProjectionT &projection, const Span &active_span,
    Point2D rover_position, double rover_yaw_rad, double speed_mps) {
  require_finite(rover_position.x, "rover_position.x");
  require_finite(rover_position.y, "rover_position.y");
  require_finite(rover_yaw_rad, "rover_yaw_rad");
  require_finite(projection.progress_s, "progress_s");
  require_finite(projection.signed_cross_track_m, "signed_cross_track_m");

  const double lookahead_distance = adaptive_lookahead_m(
      cfg.lookahead, speed_mps, projection.signed_cross_track_m);
  const auto target = geometry.lookahead_target(
      projection.progress_s, lookahead_distance, active_span);

  double local_path_heading = wrap_angle(rover_yaw_rad);
  if (projection.segment_index) {
    const int index = *projection.segment_index;
    if (index < 0 || static_cast<size_t>(index) >= geometry.segments.size())
      throw std::invalid_argument("projection.segment_index outside geometry");
    local_path_heading = geometry.segments[static_cast<size_t>(index)].heading_rad;
  } else if (target.heading_rad) {
    require_finite(*target.heading_rad, "target.heading_rad");
    local_path_heading = *target.heading_rad;
  }

  Point2D steering_target = target.point;
  double extension = 0.0;
  const double requested_s = projection.progress_s + lookahead_distance;
  require_finite(requested_s, "requested_target_s");
  const double missing = requested_s - active_span.stop_s;
  if (missing > 1e-12 && active_span.last_segment_index) {
    const int index = *active_span.last_segment_index;
    if (index < 0 || static_cast<size_t>(index) >= geometry.segments.size())
      throw std::invalid_argument("last_segment_index outside geometry");
    const double heading = geometry.segments[static_cast<size_t>(index)].heading_rad;
    steering_target.x += missing * std::cos(heading);
    steering_target.y += missing * std::sin(heading);
    extension = missing;
  }

  const double delta_x = steering_target.x - rover_position.x;
  const double delta_y = steering_target.y - rover_position.y;
  const double target_distance = std::hypot(delta_x, delta_y);
  const bool zero_vector = target_distance <= 1e-12;
  const double desired_bearing = wrap_angle(
      zero_vector ? local_path_heading : std::atan2(delta_y, delta_x));
  const double heading_error = wrap_heading_error(desired_bearing, rover_yaw_rad);
  const double path_error = wrap_heading_error(local_path_heading, rover_yaw_rad);
  const auto [limited_bearing, bearing_clamped] = limit_bearing_to_moving_cone(
      desired_bearing, rover_yaw_rad, cfg.moving_bearing_cone_rad);
  const double correction = wrap_angle(limited_bearing - local_path_heading);
  const double forward_component = delta_x * std::cos(rover_yaw_rad) +
                                   delta_y * std::sin(rover_yaw_rad);

  return {lookahead_distance, target.s, target.segment_index, target.point,
          steering_target, target_distance, extension > 0.0, extension,
          forward_component < -1e-12, desired_bearing, local_path_heading,
          path_error, heading_error, projection.signed_cross_track_m,
          desired_bearing, limited_bearing, correction, bearing_clamped,
          zero_vector};
}
} // namespace dyx::rpp
