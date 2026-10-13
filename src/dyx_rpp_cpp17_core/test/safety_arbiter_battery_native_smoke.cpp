#include "dyx_rpp_cpp17_core/safety_arbiter.hpp"
#include <cassert>
using namespace dyx::rpp;
static SafetyInput good(double t,double v=25){
 SafetyInput s;s.now_sec=t;s.mission_running=true;s.operator_motion_enabled=true;
 s.emergency_stop=false;s.px4_offboard_armed=true;s.estimator_valid=true;s.px4_age_sec=0;
 s.obstacle_clear=true;s.obstacle_age_sec=0;s.battery_voltage_v=v;s.battery_sample_time_sec=t;
 s.gnss_fix_type=6;s.gnss_sample_time_sec=t;s.correction_stream_healthy=true;
 s.correction_age_sec=0;return s;
}
int main(){
 NativeSafetyArbiter a;
 for(int t=0;t<=30;++t){auto r=a.step(good(t,21.9));
   if(t<30){assert(r.motion_allowed&&r.battery_debounce_pending);
     assert(r.battery_report_valid&&r.battery_report_percent==11.0);}
   else{assert(r.hold==SafetyHold::battery_critical&&r.battery_critical_latched);
     assert(r.rc_allowed&&r.stop_allowed&&r.telemetry_must_remain_live);
     assert(!r.start_resume_next_skip_allowed);}
 }
 assert(a.step(good(31,25)).hold==SafetyHold::battery_critical);
 assert(!a.acknowledge_battery_replacement(true,false));
 assert(a.acknowledge_battery_replacement(true,true));
 assert(a.step(good(32)).hold==SafetyHold::manual_ack_required);
 assert(a.acknowledge_manual_hold(true,true));assert(a.step(good(33)).motion_allowed);
 NativeSafetyArbiter gap;
 for(int t=0;t<10;++t)assert(gap.step(good(t,21.9)).motion_allowed);
 auto r=gap.step(good(30,21.9));assert(!r.battery_critical_latched&&r.motion_allowed);
 auto stale=good(31);stale.battery_sample_time_sec=0;
 assert(gap.step(stale).hold==SafetyHold::battery_stale);
 assert(gap.step(good(32)).hold==SafetyHold::manual_ack_required);
 NativeSafetyArbiter replay;
 assert(replay.step(good(0)).motion_allowed);
 assert(replay.step(good(1)).motion_allowed);
 auto old=good(1.5);old.battery_sample_time_sec=0.9;
 assert(replay.step(old).hold==SafetyHold::battery_stale);
 return 0;
}
