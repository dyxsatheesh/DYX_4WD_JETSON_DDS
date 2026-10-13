#include "dyx_rpp_cpp17_core/safety_mission_bridge.hpp"
#include <cassert>
using namespace dyx::rpp;
static SafetyInput good(double t) {
  SafetyInput s; s.now_sec=t;s.mission_running=true;s.operator_motion_enabled=true;
  s.emergency_stop=false;s.px4_offboard_armed=true;s.estimator_valid=true;
  s.px4_age_sec=0;s.obstacle_clear=true;s.obstacle_age_sec=0;
  s.battery_voltage_v=25;s.battery_sample_time_sec=t;
  s.gnss_fix_type=6;s.gnss_sample_time_sec=t;
  s.correction_stream_healthy=true;s.correction_age_sec=0;
  return s;
}
static MissionSequenceInput tick(double t){MissionSequenceInput s;s.monotonic_time_sec=t;
  s.emergency_stop=false;s.measured_forward_speed_mps=0.2;return s;}
int main(){
  NativeSafetyMissionBridge b;
  b.load({{0,0,"P0001","RUN:1:P1"},{80,1,"P0002","RUN:1:P2"}},
         MissionMode::manual,true,true);
  assert(b.step(good(0),tick(0)).mission.stage==MissionStage::ready);
  assert(b.start(0));
  assert(b.step(good(0.1),tick(0.1)).preview_forward_permitted);
  auto s=good(1);s.obstacle_clear=false;
  auto stop=b.step(s,tick(1));
  assert(stop.safety.hold==SafetyHold::obstacle_detected);
  assert(stop.mission.stage==MissionStage::paused && stop.mission.hold_zero);
  assert(!stop.float_pause_owned && !stop.preview_forward_permitted);
  auto fixed=b.step(good(2),tick(2));
  assert(fixed.safety.hold==SafetyHold::manual_ack_required);
  assert(!fixed.auto_resumed_float && fixed.mission.stage==MissionStage::paused);
  assert(!b.resume_manually(2.01));
  assert(!b.acknowledge_manual_hold(true,false));
  assert(b.acknowledge_manual_hold(true,true));
  assert(!b.resume_manually(2.01)); // gate invalidated by acknowledgment
  auto healthy=b.step(good(3),tick(3));
  assert(healthy.safety.motion_allowed && !healthy.auto_resumed_float);
  assert(healthy.mission.stage==MissionStage::paused);
  assert(b.resume_manually(3.1));
  assert(b.step(good(3.2),tick(3.2)).preview_forward_permitted);
  b.pause();assert(b.mission().stage()==MissionStage::paused);
  assert(!b.float_pause_owned());
  assert(b.step(good(3.3),tick(3.3)).mission.hold_zero);
  assert(b.resume_manually(3.4));
  assert(b.step(good(3.5),tick(3.5)).preview_forward_permitted);
  s=good(4);s.emergency_stop=true;
  auto estop=b.step(s,tick(4));
  assert(estop.safety.hold==SafetyHold::emergency_stop);
  assert(estop.mission.stage==MissionStage::fault);
  assert(!estop.preview_forward_permitted && !estop.rc_allowed);
  assert(estop.stop_allowed && estop.telemetry_required);
  assert(!b.resume_manually(4.01));
  b.stop();assert(b.mission().stage()==MissionStage::stopped);
  NativeSafetyMissionBridge clock;
  clock.load({{0,0,"P0001","CLOCK:P1"}},MissionMode::automatic,true,true);
  auto mismatch=clock.step(good(0),tick(0.2));
  assert(!mismatch.preview_forward_permitted);
  assert(clock.mission().stage()==MissionStage::stopped);
  assert(!clock.start(0.2));
  return 0;
}
