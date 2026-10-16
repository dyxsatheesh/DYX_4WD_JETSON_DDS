#include "dyx_rpp_cpp17_core/safety_mission_bridge.hpp"
#include <cassert>
using namespace dyx::rpp;
static SafetyInput good(double t){
  SafetyInput s;s.now_sec=t;s.mission_running=true;s.operator_motion_enabled=true;
  s.emergency_stop=false;s.px4_offboard_armed=true;s.estimator_valid=true;s.px4_age_sec=0;
  s.obstacle_clear=true;s.obstacle_age_sec=0;s.battery_voltage_v=25;s.battery_sample_time_sec=t;
  s.gnss_fix_type=6;s.gnss_sample_time_sec=t;s.correction_stream_healthy=true;s.correction_age_sec=0;
  return s;
}
static MissionSequenceInput frame(double t){
  MissionSequenceInput m;m.monotonic_time_sec=t;m.emergency_stop=false;
  m.terminal_present=true;m.terminal_identity="RUN:H:P1";
  m.terminal_certificate_valid=true;m.terminal_zero_latched=true;
  m.terminal_stationary=true;m.terminal_radial_error_m=0.01;
  m.measured_forward_speed_mps=0;return m;
}
int main(){
  MissionSequenceConfig mc;mc.marking_hold_sec=20.0;mc.raw_gnss_timeout_sec=20.0;
  NativeSafetyMissionBridge bridge(SafetyConfig{},mc);
  bridge.load({{0,0,"P0001","RUN:H:P1"}},MissionMode::automatic,true,true);
  bridge.step(good(0),frame(0));assert(bridge.start(0));
  auto hold=bridge.step(good(0.1),frame(0.1));
  assert(hold.mission.stage==MissionStage::verify_hold && hold.mission.hold_zero);
  for(int i=1;i<=5;++i){auto s=good(i);s.gnss_fix_type=5;
    auto o=bridge.step(s,frame(i));
    assert(o.mission.stage==MissionStage::paused && !o.preview_forward_permitted);
  }
  auto f=good(6);f.gnss_fix_type=5;
  auto paused=bridge.step(f,frame(6));
  assert(paused.mission.stage==MissionStage::paused && paused.mission.hold_zero);
  assert(!paused.float_pause_owned); // do not auto-resume while verifying marking
  auto fix=bridge.step(good(7),frame(7));
  assert(!fix.safety.auto_resume_rtk_float && !fix.auto_resumed_float);
  assert(fix.mission.stage==MissionStage::paused);
  assert(!fix.preview_forward_permitted);
  assert(bridge.resume_manually(7.1)); // new operator decision required
  auto resumed=bridge.step(good(7.2),frame(7.2));
  assert(resumed.mission.stage==MissionStage::verify_hold);
  assert(!resumed.preview_forward_permitted); // certified stop, no automatic motion
  return 0;
}
