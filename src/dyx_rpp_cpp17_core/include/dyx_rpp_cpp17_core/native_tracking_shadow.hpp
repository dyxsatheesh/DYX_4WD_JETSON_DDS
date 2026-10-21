#pragma once
// D2B: bounded, non-actuating ENU polyline tracking diagnostics.
// Never grants mission authority or writes a speed/yaw-rate command.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace dyx::rpp {
struct TrackingShadowInput {
  double east_m{0.0}, north_m{0.0}, yaw_enu_rad{0.0};
  double lookahead_m{0.40};
  bool position_fresh{false};
  bool rear_reference_calibrated{false};
};
struct TrackingShadowResult {
  bool geometry_valid{false};
  bool hold_zero{true};
  bool motion_authorized{false};
  const char* reason{"NO_VERIFIED_PATH"};
  const char* segment_kind{"UNKNOWN"};
  double cross_track_m{0.0};
  double remaining_m{0.0};
  double lookahead_east_m{0.0}, lookahead_north_m{0.0};
  double bearing_error_rad{0.0};
  double curve_heading_change_deg{0.0};
};
namespace tracking_shadow_detail {
constexpr double pi = 3.14159265358979323846;
inline double wrap(double x) noexcept { return std::remainder(x, 2.0*pi); }
inline bool finite(double x) noexcept { return std::isfinite(x); }
}
// RawPath is the existing PreparedPathSnapshot::raw_navigation() vector;
// its entries expose .point.x/.point.y in local ENU metres.
// Work per tick is bounded. For longer legs return HOLD/oversize rather than
// consuming unbounded time in the 50 Hz ROS mission executor.
template <typename RawPath>
TrackingShadowResult evaluate_tracking_shadow(const RawPath& raw,
    std::size_t start, std::size_t stop, const TrackingShadowInput& in) noexcept {
  using namespace tracking_shadow_detail;
  TrackingShadowResult out{};
  if (!finite(in.east_m) || !finite(in.north_m) ||
      !finite(in.yaw_enu_rad) || !finite(in.lookahead_m) ||
      in.lookahead_m < 0.05 || in.lookahead_m > 3.0) {
    out.reason = "INVALID_POSE_OR_LOOKAHEAD"; return out;
  }
  if (start >= stop || stop >= raw.size()) {
    out.reason = "INVALID_ACTIVE_SPAN"; return out;
  }
  if (stop - start > 2048) {
    out.reason = "SPAN_OVER_BUDGET"; return out;
  }
  double best_d2 = std::numeric_limits<double>::infinity();
  std::size_t nearest = start;
  double nearest_t = 0.0, cross = 0.0, total = 0.0;
  for (std::size_t i = start; i < stop; ++i) {
    const auto& a = raw[i].point;
    const auto& b = raw[i+1].point;
    if (!finite(a.x) || !finite(a.y) || !finite(b.x) || !finite(b.y)) {
      out.reason = "NONFINITE_PATH"; return out;
    }
    const double dx = b.x-a.x, dy = b.y-a.y;
    const double len2 = dx*dx + dy*dy;
    if (!finite(len2) || len2 < 1e-12) {
      out.reason = "ZERO_LENGTH_SEGMENT"; return out;
    }
    const double len = std::sqrt(len2);
    total += len;
    if (!finite(total)) { out.reason = "PATH_OVERFLOW"; return out; }
    const double t = std::clamp(((in.east_m-a.x)*dx +
         (in.north_m-a.y)*dy)/len2, 0.0, 1.0);
    const double ox = in.east_m - (a.x+t*dx);
    const double oy = in.north_m - (a.y+t*dy);
    const double dist2 = ox*ox+oy*oy;
    if (!finite(dist2)) {out.reason="DISTANCE_OVERFLOW";return out;}
    if (dist2 < best_d2) {
      best_d2 = dist2; nearest=i; nearest_t=t;
      cross=(dx*oy-dy*ox)/len;
    }
  }
  double remaining=0.0;
  for (std::size_t i=nearest; i<stop; ++i) {
    const auto& a=raw[i].point;
    const auto& b=raw[i+1].point;
    const double len=std::hypot(b.x-a.x,b.y-a.y);
    remaining += len*(i==nearest ? 1.0-nearest_t : 1.0);
  }
  double to_go = in.lookahead_m;
  double target_e=raw[stop].point.x, target_n=raw[stop].point.y;
  double first_heading=0.0, last_heading=0.0;
  bool first=true;
  for(std::size_t i=nearest;i<stop;++i) {
    const auto& a=raw[i].point;
    const auto& b=raw[i+1].point;
    const double dx=b.x-a.x,dy=b.y-a.y;
    const double len=std::hypot(dx,dy);
    const double heading=std::atan2(dy,dx);
    if(first){first_heading=heading;first=false;}
    const double offset=(i==nearest?nearest_t:0.0);
    const double available=len*(1.0-offset);
    if(to_go<=available) {
      const double t=offset+to_go/len;
      target_e=a.x+t*dx; target_n=a.y+t*dy;
      last_heading=heading;
      break;
    }
    to_go-=available;
    last_heading=heading;
  }
  const double curve=std::abs(wrap(last_heading-first_heading))*180.0/dyx::rpp::pi;
  const double desired=std::atan2(target_n-in.north_m,target_e-in.east_m);
  out.geometry_valid=true;
  out.cross_track_m=cross;
  out.remaining_m=remaining;
  out.lookahead_east_m=target_e;
  out.lookahead_north_m=target_n;
  out.bearing_error_rad=wrap(desired-in.yaw_enu_rad);
  out.curve_heading_change_deg=curve;
  out.segment_kind=curve>=5.0?"CURVE":"STRAIGHT";
  out.reason= !in.position_fresh?"POSITION_STALE":
      !in.rear_reference_calibrated?"REAR_REFERENCE_UNCALIBRATED":
      "GEOMETRY_PREVIEW_ONLY";
  // Even fully calibrated preview never becomes a command capability.
  return out;
}
} // namespace dyx::rpp
