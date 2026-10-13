#include "dyx_rpp_cpp17_core/safety_arbiter.hpp"
#include <cassert>
using namespace dyx::rpp;
static SafetyInput good(double t){
 SafetyInput s;s.now_sec=t;s.mission_running=true;s.operator_motion_enabled=true;
 s.emergency_stop=false;s.px4_offboard_armed=true;s.estimator_valid=true;s.px4_age_sec=0;
 s.obstacle_clear=true;s.obstacle_age_sec=0;s.battery_voltage_v=25;s.battery_sample_time_sec=t;
 s.gnss_fix_type=6;s.gnss_sample_time_sec=t;s.correction_stream_healthy=true;
 s.correction_age_sec=0;return s;
}
int main(){
 assert(!NativeSafetyArbiter{}.step(SafetyInput{}).motion_allowed);
 NativeSafetyArbiter a;assert(a.step(good(0)).motion_allowed);
 auto s=good(1);s.obstacle_clear=false;
 assert(a.step(s).hold==SafetyHold::obstacle_detected);
 assert(a.step(good(2)).hold==SafetyHold::manual_ack_required);
 assert(!a.acknowledge_manual_hold(false,true));
 assert(a.acknowledge_manual_hold(true,true));assert(a.step(good(3)).motion_allowed);
 s=good(4);s.correction_stream_healthy=false;
 assert(a.step(s).hold==SafetyHold::corrections_stale);
 assert(a.step(good(5)).hold==SafetyHold::manual_ack_required);
 assert(a.acknowledge_manual_hold(true,true));assert(a.step(good(6)).motion_allowed);
 s=good(7);s.px4_age_sec=1;assert(a.step(s).hold==SafetyHold::px4_stale);
 s=good(8);s.emergency_stop=true;
 auto e=a.step(s);assert(e.hold==SafetyHold::emergency_stop&&!e.rc_allowed);
 assert(e.stop_allowed&&e.telemetry_must_remain_live);
 assert(a.step(good(9)).hold==SafetyHold::emergency_stop);
 assert(!a.release_emergency_stop(true,false,true));
 assert(!a.release_emergency_stop(true,true,false));
 assert(a.release_emergency_stop(true,true,true));
 assert(a.step(good(10)).hold==SafetyHold::manual_ack_required);
 assert(a.acknowledge_manual_hold(true,true));assert(a.step(good(11)).motion_allowed);
 assert(a.step(good(10.5)).hold==SafetyHold::bad_clock);
 NativeSafetyArbiter b;auto disabled=good(0);disabled.operator_motion_enabled=false;
 assert(b.step(disabled).hold==SafetyHold::disabled);
 auto invalid=good(1);invalid.estimator_valid=false;
 assert(b.step(invalid).hold==SafetyHold::estimator_invalid);
 return 0;
}
