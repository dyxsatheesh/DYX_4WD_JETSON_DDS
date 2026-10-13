#include "dyx_rpp_cpp17_core/safety_mission_bridge.hpp"
#include <cassert>
#include <string>
using namespace dyx::rpp;
static SafetyInput good(double t) {
  SafetyInput s;
  s.now_sec=t; s.mission_running=true; s.operator_motion_enabled=true;
  s.emergency_stop=false; s.px4_offboard_armed=true; s.estimator_valid=true;
  s.px4_age_sec=0.0; s.obstacle_clear=true; s.obstacle_age_sec=0.0;
  s.battery_voltage_v=25.0; s.battery_sample_time_sec=t;
  s.gnss_fix_type=6; s.gnss_sample_time_sec=t;
  s.correction_stream_healthy=true; s.correction_age_sec=0.1;
  return s;
}
static MissionSequenceInput frame(double t) {
  MissionSequenceInput m;
  m.monotonic_time_sec=t; m.emergency_stop=false;
  m.motion_health_authorized=false; m.measured_forward_speed_mps=0.4;
  return m;
}
int main() {
  NativeSafetyMissionBridge b;
  b.load({{0,0,"P0001","RUN:A:P0001"},{100,1,"P0002","RUN:A:P0002"}},
         MissionMode::automatic,true,true);
  assert(!b.start(0.0));  // no fresh health evidence
  auto first=b.step(good(0.0), frame(0.0));
  assert(first.mission.stage==MissionStage::ready);
  assert(!b.start(0.5));  // expired control gate
  assert(b.start(0.1));
  auto forward=b.step(good(0.2),frame(0.2));
  assert(forward.preview_forward_permitted && !forward.mission.hold_zero);
  for (int t=1;t<=5;++t) {
    auto s=good(static_cast<double>(t));s.gnss_fix_type=5;
    auto x=b.step(s,frame(s.now_sec));
    assert(x.safety.rtk_float_confirming);
    assert(x.preview_forward_permitted);
    assert(x.mission.stage==MissionStage::approaching);
  }
  auto s=good(6.0);s.gnss_fix_type=5;
  auto paused=b.step(s,frame(6.0));
  assert(paused.safety.hold==SafetyHold::rtk_float_paused);
  assert(paused.mission.stage==MissionStage::paused && paused.mission.hold_zero);
  assert(paused.float_pause_owned && !paused.preview_forward_permitted);
  assert(!b.resume_manually(6.1));  // automatic recovery only after RTK FIXED
  s=good(6.2);s.gnss_fix_type=5;
  auto still_paused=b.step(s,frame(6.2));
  assert(still_paused.mission.stage==MissionStage::paused);
  s=good(7.0);
  const auto recovered=b.step(s,frame(7.0));
  assert(recovered.safety.motion_allowed && recovered.safety.auto_resume_rtk_float);
  assert(recovered.auto_resumed_float && !recovered.float_pause_owned);
  assert(recovered.mission.stage==MissionStage::approaching);
  assert(recovered.preview_forward_permitted && !recovered.mission.hold_zero);
  assert(!b.start(7.01)); // cannot restart an already running mission
  b.stop();
  assert(b.mission().stage()==MissionStage::stopped);
  assert(!b.start(7.02));
  return 0;
}
