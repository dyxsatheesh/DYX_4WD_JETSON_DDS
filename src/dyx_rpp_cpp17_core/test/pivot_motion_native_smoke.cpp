#include "dyx_rpp_cpp17_core/pivot_motion.hpp"
#include <stdexcept>
using namespace dyx::rpp;
static void require(bool v,const char *name){if(!v)throw std::runtime_error(name);}
int main(){
  VerifiedPivotStateMachine x;
  PivotMotionInput s{};s.telemetry_fresh=true;s.anchor_radial_error_m=0.005;
  s.heading_error_rad=1.0;s.pivot_requested=true;s.dt_sec=0.05;
  auto r=x.step(s);require(r.state==MotionState::CORNER_APPROACH,"corner");
  s.monotonic_time_sec=0.05;s.brake_to_anchor_requested=true;
  r=x.step(s);require(r.state==MotionState::BRAKE_TO_ANCHOR,"brake");
  s.monotonic_time_sec=0.10;r=x.step(s);require(r.state==MotionState::STOP_SETTLE,"stop");
  s.monotonic_time_sec=0.15;r=x.step(s);require(r.directive==MotionDirective::HOLD_ZERO,"stop zero");
  for(int i=4;i<=8 && r.state!=MotionState::PIVOT;i++){
    s.monotonic_time_sec=i*0.05;r=x.step(s);
  }
  require(r.state==MotionState::PIVOT,"pivot entered");
  s.telemetry_fresh=false;s.monotonic_time_sec+=0.05;
  r=x.step(s);require(r.directive==MotionDirective::HOLD_ZERO,"stale inhibits pivot");
  x.reset(10.0);require(x.state()==MotionState::TRACK,"reset");
  bool invalid=false;
  try{x.reset(-1.0);}catch(const std::invalid_argument&){invalid=true;}
  require(invalid,"negative reset rejected");
  return 0;
}
