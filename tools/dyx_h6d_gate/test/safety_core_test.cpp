// Tests the real existing ROS-free native safety arbiter, no hardware access.
#include "dyx_rpp_cpp17_core/safety_arbiter.hpp"
#include <cassert>
#include <cmath>
#include <string>
using namespace dyx::rpp;
static SafetyInput nominal(double t, double voltage, int fix) {
  SafetyInput x;
  x.now_sec=t;
  x.mission_running=true;
  x.operator_motion_enabled=true;
  x.emergency_stop=false;
  x.px4_offboard_armed=true;
  x.estimator_valid=true;
  x.px4_age_sec=0.01;
  x.obstacle_clear=true;
  x.obstacle_age_sec=0.01;
  x.battery_voltage_v=voltage;
  x.battery_sample_time_sec=t;
  x.gnss_fix_type=fix;
  x.gnss_sample_time_sec=t;
  x.correction_stream_healthy=true;
  x.correction_age_sec=0.01;
  return x;
}
int main(){
  {
    NativeSafetyArbiter a;
    auto x=nominal(1.0,24.0,6);
    assert(a.step(x).motion_allowed);
    x=nominal(2.0,24.0,5);
    const auto first=a.step(x);
    assert(first.rtk_float_confirming);
    assert(!first.battery_critical_latched);
    for (int i=3;i<=6;i++) a.step(nominal(double(i),24.0,5));
    auto pause=a.step(nominal(7.0,24.0,5));
    assert(pause.hold==SafetyHold::rtk_float_paused);
    assert(!pause.motion_allowed);
    auto fix=a.step(nominal(8.0,24.0,6));
    assert(fix.auto_resume_rtk_float);
  }
  {
    NativeSafetyArbiter a;
    a.step(nominal(1.0,24.0,6));
    auto first=a.step(nominal(10.0,22.0,6));
    assert(first.battery_debounce_pending && !first.battery_critical_latched);
    for (int i=11;i<40;i++){
      auto v=a.step(nominal(double(i),22.0,6));
      assert(!v.battery_critical_latched);
    }
    auto threshold=a.step(nominal(40.0,22.0,6));
    assert(threshold.battery_critical_latched);
    assert(threshold.hold==SafetyHold::battery_critical);
    assert(!threshold.motion_allowed);
    auto recoveredVoltage=a.step(nominal(41.0,24.0,6));
    assert(recoveredVoltage.battery_critical_latched); // explicit ack required
  }
  {
    NativeSafetyArbiter a;
    auto x=nominal(1.0,24.0,6);
    x.correction_stream_healthy=false;
    assert(!a.step(x).motion_allowed);
    NativeSafetyArbiter b;
    auto y=nominal(1.0,24.0,6);
    y.obstacle_age_sec=1.0;
    assert(!b.step(y).motion_allowed);
  }
  return 0;
}
