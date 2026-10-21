#pragma once
// D2A: geometric reverse-then-pivot mission handoff. Advisory only.
// Marking reference Pn remains fixed while body pivot centre is backed onto Pn.
// The trusted mission controller must independently certify stopped motion,
// calibration, RTK and physical safety before using any guidance.
#include "dyx_rpp_cpp17_core/primitives.hpp"
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <limits>

namespace dyx::rpp {
enum class HandoffPhase { HOLD, DIRECT_FORWARD, REVERSE_TO_PIVOT, STATIONARY_PIVOT };
inline const char* handoff_phase_name(HandoffPhase p) noexcept {
 switch(p){case HandoffPhase::HOLD:return "HOLD";case HandoffPhase::DIRECT_FORWARD:return "DIRECT_FORWARD";
 case HandoffPhase::REVERSE_TO_PIVOT:return "REVERSE_TO_PIVOT";
 case HandoffPhase::STATIONARY_PIVOT:return "STATIONARY_PIVOT";}return "HOLD";
}
struct ReversePivotHandoffInput {
 Point2D marked_point_enu{};
 Point2D next_point_enu{};
 Point2D rear_master_now_enu{};
 double current_heading_enu_rad{std::numeric_limits<double>::quiet_NaN()};
 // Distance pivot centre ahead of rear master along vehicle forward axis.
 double rear_master_to_pivot_forward_m{0.0};
 bool marking_resolved{false};
 bool position_fresh{false};
 bool reference_calibrated{false};
 bool pivot_offset_calibrated{false};
 bool stop_certified{false};
 bool physical_safety_verified{false};
};
struct ReversePivotHandoffResult {
 HandoffPhase phase{HandoffPhase::HOLD};
 const char* reason{"NOT_READY"};
 bool geometry_valid{false};
 bool action_authorized{false}; // Never true in D2A.
 double heading_change_deg{0.0};
 double desired_heading_enu_rad{0.0};
 double reverse_distance_m{0.0};
 double pivot_centre_error_m{0.0};
};
inline ReversePivotHandoffResult plan_reverse_pivot_handoff(const ReversePivotHandoffInput& in) noexcept {
 ReversePivotHandoffResult o;
 for(const double v:{in.marked_point_enu.x,in.marked_point_enu.y,in.next_point_enu.x,
   in.next_point_enu.y,in.rear_master_now_enu.x,in.rear_master_now_enu.y,
   in.current_heading_enu_rad,in.rear_master_to_pivot_forward_m})
   if(!std::isfinite(v)){o.reason="NONFINITE_GEOMETRY";return o;}
 if(in.rear_master_to_pivot_forward_m<0.0||in.rear_master_to_pivot_forward_m>2.0){o.reason="INVALID_PIVOT_OFFSET";return o;}
 const double dx=in.next_point_enu.x-in.marked_point_enu.x;
 const double dy=in.next_point_enu.y-in.marked_point_enu.y;
 if(!std::isfinite(std::hypot(dx,dy))||std::hypot(dx,dy)<0.001){o.reason="NEXT_GOAL_COLLAPSED";return o;}
 o.desired_heading_enu_rad=std::atan2(dy,dx);
 constexpr double kPi=3.14159265358979323846;
 const double delta=std::remainder(o.desired_heading_enu_rad-in.current_heading_enu_rad,2.0*kPi);
 o.heading_change_deg=delta*180.0/kPi;
 const double fx=std::cos(in.current_heading_enu_rad),fy=std::sin(in.current_heading_enu_rad);
 const double pivot_x=in.rear_master_now_enu.x+fx*in.rear_master_to_pivot_forward_m;
 const double pivot_y=in.rear_master_now_enu.y+fy*in.rear_master_to_pivot_forward_m;
 const double offset_along=(pivot_x-in.marked_point_enu.x)*fx+(pivot_y-in.marked_point_enu.y)*fy;
 const double lateral_error=std::abs(-(pivot_x-in.marked_point_enu.x)*fy+(pivot_y-in.marked_point_enu.y)*fx);
 o.reverse_distance_m=offset_along>0.0?offset_along:0.0;
 o.pivot_centre_error_m=std::hypot(pivot_x-in.marked_point_enu.x,pivot_y-in.marked_point_enu.y);
 o.geometry_valid=true;
 if(!in.marking_resolved){o.reason="MARKING_NOT_RESOLVED";return o;}
 if(!in.position_fresh||!in.reference_calibrated||!in.pivot_offset_calibrated){o.reason="POSITION_OR_CALIBRATION_UNVERIFIED";return o;}
 if(!in.stop_certified||!in.physical_safety_verified){o.reason="STOP_OR_SAFETY_UNVERIFIED";return o;}
 if(std::abs(o.heading_change_deg)<15.0){o.phase=HandoffPhase::DIRECT_FORWARD;o.reason="UNDER_15_DEGREES";return o;}
 if(lateral_error>0.020){o.reason="PIVOT_LATERAL_MISALIGNMENT";return o;}
 if(offset_along< -0.020){o.reason="PIVOT_BEHIND_MARK";return o;}
 if(o.pivot_centre_error_m>0.020){o.phase=HandoffPhase::REVERSE_TO_PIVOT;o.reason="BACK_TO_MARKED_POINT";return o;}
 o.phase=HandoffPhase::STATIONARY_PIVOT;o.reason="PIVOT_AT_MARKED_POINT";return o;
}
} // namespace dyx::rpp
