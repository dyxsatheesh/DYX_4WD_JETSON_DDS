#pragma once
#include "dyx_rpp_cpp17_core/primitives.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace dyx::rpp {
struct GuidancePreview {double heading_error_rad{},forward_mps{},yaw_rate_enu_radps{};bool pivot_needed{};};
inline GuidancePreview preview_guidance(double east,double north,double yaw_enu,
    double target_east,double target_north,double along,double total,
    double cruise=0.6,double accel_distance=0.8,double decel_distance=0.8){
  for(double v:{east,north,yaw_enu,target_east,target_north,along,total,cruise,accel_distance,decel_distance})
    if(!std::isfinite(v))throw std::invalid_argument("Nonfinite guidance argument");
  if(total<=0.0||along<0.0||cruise<0.0||cruise>1.0||accel_distance<=0.0||decel_distance<=0.0)
    throw std::invalid_argument("Invalid guidance limits");
  const double dx=target_east-east,dy=target_north-north;
  if(std::hypot(dx,dy)<1e-9)return {};
  const double error=wrap_angle(std::atan2(dy,dx)-yaw_enu);
  const bool pivot=std::abs(error)>=15.0*pi/180.0;
  const double ramp=std::clamp(std::min(along/accel_distance,(total-along)/decel_distance),0.0,1.0);
  // Diagnostic only. Near a terminal marking a production controller requires
  // a separate verified stop profile and a trusted active semantic span.
  const double forward=pivot?0.0:cruise*ramp;
  const double rate=std::clamp((pivot?1.2:0.85)*error,-(pivot?0.18:0.15),(pivot?0.18:0.15));
  return {error,forward,rate,pivot};
}
}
