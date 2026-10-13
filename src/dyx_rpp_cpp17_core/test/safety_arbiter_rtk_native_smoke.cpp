#include "dyx_rpp_cpp17_core/safety_arbiter.hpp"
#include <cassert>
using namespace dyx::rpp;
static SafetyInput good(double t) {
  SafetyInput s;s.now_sec=t;s.mission_running=true;s.operator_motion_enabled=true;
  s.emergency_stop=false;s.px4_offboard_armed=true;s.estimator_valid=true;s.px4_age_sec=0;
  s.obstacle_clear=true;s.obstacle_age_sec=0;s.battery_voltage_v=25;
  s.battery_sample_time_sec=t;s.gnss_fix_type=6;s.gnss_sample_time_sec=t;
  s.correction_stream_healthy=true;s.correction_age_sec=0.2;return s;
}
int main(){
 NativeSafetyArbiter a;
 assert(a.step(good(0)).motion_allowed);
 for(int t=1;t<=6;++t){auto s=good(t);s.gnss_fix_type=5;auto r=a.step(s);
   if(t<6){assert(r.motion_allowed&&r.rtk_float_confirming);}
   else assert(r.hold==SafetyHold::rtk_float_paused&&!r.motion_allowed);
 }
 auto recovered=a.step(good(7));assert(recovered.motion_allowed&&recovered.auto_resume_rtk_float);
 assert(a.step(good(8)).motion_allowed);
 auto s=good(9);s.gnss_fix_type=4;
 assert(a.step(s).hold==SafetyHold::rtk_lost);
 assert(a.step(good(10)).hold==SafetyHold::manual_ack_required);
 assert(!a.acknowledge_manual_hold(true,false));
 assert(a.acknowledge_manual_hold(true,true));
 assert(a.step(good(11)).motion_allowed);
 s=good(12);s.gnss_sample_time_sec=8;
 assert(a.step(s).hold==SafetyHold::gnss_stale);
 assert(a.step(good(13)).hold==SafetyHold::manual_ack_required);
 NativeSafetyArbiter gap;assert(gap.step(good(0)).motion_allowed);
 s=good(1);s.gnss_fix_type=5;assert(gap.step(s).motion_allowed);
 s=good(6);s.gnss_fix_type=5;assert(gap.step(s).motion_allowed); // no 5 sec proof
 s=good(7);s.gnss_fix_type=5;assert(gap.step(s).motion_allowed);
 s=good(8);s.gnss_sample_time_sec=6.9;
 assert(gap.step(s).hold==SafetyHold::gnss_stale); // out-of-order
 return 0;
}
