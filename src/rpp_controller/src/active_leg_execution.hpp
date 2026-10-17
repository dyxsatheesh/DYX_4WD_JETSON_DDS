#pragma once
#include "dyx_rpp_cpp17_core/path_index.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace dyx::rpp {
struct ActiveLegControlPreview {
  std::string goal_identity;
  std::string phase;
  double cross_track_m{}, radial_m{}, along_m{}, remaining_m{};
  double lookahead_east_m{}, lookahead_north_m{};
  double heading_error_rad{}, forward_preview_mps{}, yaw_rate_preview_radps{};
  double turn_at_previous_mark_rad{};
  double reverse_distance_preview_m{};
  bool reverse_pivot_required{}, terminal_zone{}, stationary_at_mark{}, position_hold_required{true};
};

struct ActiveLegControlConfig {
  double cruise_mps{0.6};
  double accel_m{0.8}, decel_m{0.8}, lookahead_m{0.55};
  double terminal_radial_m{0.02};
  double stationary_mps{0.03};
  double turn_threshold_rad{15.0*pi/180.0};
  double pivot_release_rad{2.0*pi/180.0};
  double move_yaw_limit_radps{0.15}, pivot_yaw_limit_radps{0.18};
  double master_to_pivot_m{0.42};
  void validate() const {
    const double vals[]={cruise_mps,accel_m,decel_m,lookahead_m,terminal_radial_m,
      stationary_mps,turn_threshold_rad,pivot_release_rad,move_yaw_limit_radps,
      pivot_yaw_limit_radps,master_to_pivot_m};
    for (double v:vals) if(!std::isfinite(v)||v<0.0) throw std::invalid_argument("NONFINITE_OR_NEGATIVE_RPP_SETTING");
    if(cruise_mps<0.15 || cruise_mps>1.0 || accel_m<=0.0 || decel_m<=0.0 ||
       lookahead_m>5.0 || terminal_radial_m<=0.0 || stationary_mps<=0.0 ||
       turn_threshold_rad>pi || pivot_release_rad>turn_threshold_rad ||
       move_yaw_limit_radps>0.15 || pivot_yaw_limit_radps>0.18 || master_to_pivot_m>2.0)
      throw std::invalid_argument("RPP_SETTING_OUT_OF_SAFE_RANGE");
  }
};

// Geometry / command PREVIEW only. Never a motor, spray, terminal certificate,
// physical reverse authorization, or PX4 offboard permission.
inline ActiveLegControlPreview plan_active_leg_preview(
    const PathGeometryIndex& path, const std::vector<std::int32_t>& marks,
    std::int32_t active_mark, Point2D rover, double yaw_enu_rad,
    double measured_speed_mps, const ActiveLegControlConfig& config={}) {
  config.validate();
  if(!std::isfinite(rover.x)||!std::isfinite(rover.y)||!std::isfinite(yaw_enu_rad)||
     !std::isfinite(measured_speed_mps)||measured_speed_mps<0.0)
    throw std::invalid_argument("INVALID_POSE_OR_SPEED");
  if(active_mark<=0 || static_cast<std::size_t>(active_mark)>=marks.size())
    throw std::invalid_argument("ACTIVE_MARK_NOT_P2_OR_LATER");
  const int first=marks.at(static_cast<std::size_t>(active_mark-1));
  const int stop=marks.at(static_cast<std::size_t>(active_mark));
  if(first<0||stop<=first||static_cast<std::size_t>(stop)>=path.raw_points.size())
    throw std::invalid_argument("INVALID_MARKING_SPAN");
  if(path.raw_points[first].point_type!=point_marking ||
     path.raw_points[stop].point_type!=point_marking ||
     path.raw_points[stop].marking_index!=active_mark)
    throw std::invalid_argument("MISSING_TRUSTED_SEMANTIC_MARKS");
  const auto span=path.active_span(stop,first);
  const auto projection=path.project(rover,span);
  const auto target=path.lookahead_target(projection.progress_s,config.lookahead_m,span);
  const auto endpoint=path.raw_points.at(static_cast<std::size_t>(stop)).point;
  const double radial=std::hypot(rover.x-endpoint.x,rover.y-endpoint.y);
  const double dx=target.point.x-rover.x,dy=target.point.y-rover.y;
  // At an exact target use the active segment's heading to avoid atan2(0,0).
  const double desired=std::hypot(dx,dy)>1e-9 ? std::atan2(dy,dx) :
    (target.heading_rad ? *target.heading_rad : yaw_enu_rad);
  const double error=wrap_angle(desired-yaw_enu_rad);
  const double along=std::clamp(projection.progress_s-span.start_s,0.0,span.stop_s-span.start_s);
  const double remaining=std::max(0.0,span.stop_s-projection.progress_s);
  const double ramp=std::clamp(std::min(along/config.accel_m,remaining/config.decel_m),0.0,1.0);
  ActiveLegControlPreview result;
  result.goal_identity=span.active_goal_identity;
  result.cross_track_m=projection.signed_cross_track_m;
  result.radial_m=radial;
  result.along_m=along;
  result.remaining_m=remaining;
  result.lookahead_east_m=target.point.x;
  result.lookahead_north_m=target.point.y;
  result.heading_error_rad=error;
  result.terminal_zone=radial<=config.terminal_radial_m;
  result.stationary_at_mark=result.terminal_zone && measured_speed_mps<=config.stationary_mps;
  result.forward_preview_mps=0.0;
  result.yaw_rate_preview_radps=0.0;
  result.phase="HOLD_UNVERIFIED";
  result.position_hold_required=true;
  // A pivot is considered at the start of the active leg (P[n-1]).
  // It needs certified center capture, not a guessed reverse command.
  if(active_mark>=2) {
    const int prev=marks.at(static_cast<std::size_t>(active_mark-2));
    const auto before=path.raw_points.at(static_cast<std::size_t>(prev)).point;
    const auto at=path.raw_points.at(static_cast<std::size_t>(first)).point;
    const auto next=path.raw_points.at(static_cast<std::size_t>(stop)).point;
    const double incoming=std::atan2(at.y-before.y,at.x-before.x);
    const double outgoing=std::atan2(next.y-at.y,next.x-at.x);
    result.turn_at_previous_mark_rad=wrap_angle(outgoing-incoming);
    result.reverse_pivot_required=std::abs(result.turn_at_previous_mark_rad)>=config.turn_threshold_rad;
    result.reverse_distance_preview_m=result.reverse_pivot_required?config.master_to_pivot_m:0.0;
  }
  if(result.stationary_at_mark){
    result.phase="AT_MARK_WAIT_RAW_GNSS_AND_SPRAY_CERTIFICATE";
    return result;
  }
  if(result.terminal_zone){
    result.phase="TERMINAL_BRAKE_AND_HOLD";
    return result;
  }
  if(result.reverse_pivot_required && along<config.master_to_pivot_m+0.05) {
    result.phase="REVERSE_CENTER_CAPTURE_AND_PIVOT_NOT_CERTIFIED";
    result.yaw_rate_preview_radps=std::clamp(0.85*error,
      -config.pivot_yaw_limit_radps,config.pivot_yaw_limit_radps);
    return result;
  }
  if(std::abs(error)>=config.turn_threshold_rad){
    result.phase="STATIONARY_ALIGNMENT_PREVIEW";
    result.yaw_rate_preview_radps=std::clamp(0.85*error,
      -config.pivot_yaw_limit_radps,config.pivot_yaw_limit_radps);
    return result;
  }
  result.phase="FORWARD_PREVIEW_ONLY";
  result.forward_preview_mps=config.cruise_mps*ramp;
  result.yaw_rate_preview_radps=std::clamp(0.85*error,
    -config.move_yaw_limit_radps,config.move_yaw_limit_radps);
  // PREVIEW only; independent safety supervision and PX4 bridge remain absent.
  return result;
}
} // namespace dyx::rpp
