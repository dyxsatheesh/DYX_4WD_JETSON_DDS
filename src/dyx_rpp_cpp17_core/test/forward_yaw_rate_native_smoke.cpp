#include "dyx_rpp_cpp17_core/forward_yaw_rate.hpp"
#include <cmath>
#include <stdexcept>
#include <string>

using namespace dyx::rpp;
static void check(bool value, const char *why) {
  if (!value) throw std::runtime_error(why);
}
static bool close(double a,double b,double tol=1.e-12) {return std::abs(a-b)<=tol;}
int main() {
  ForwardYawRatePlanner f;
  ForwardYawInput s;
  s.now_sec=0.; s.dt_sec=0.05; s.motion_enabled=true; s.telemetry_fresh=true;
  s.requested_forward_speed_mps=0.6;
  auto o=f.step(s);
  check(o.phase==ForwardYawPhase::TRACK && close(o.velocity_east_mps,0.6),"east forward");
  s.now_sec=0.05; s.rover_yaw_enu_rad=pi/2; s.target_yaw_enu_rad=pi/2;
  o=f.step(s);
  check(close(o.velocity_north_mps,0.6),"north forward");
  s.now_sec=0.10; s.rover_yaw_enu_rad=0; s.target_yaw_enu_rad=20*pi/180;
  o=f.step(s);
  check(o.phase==ForwardYawPhase::WAIT_PIVOT && close(o.forward_speed_mps,0),"permission block");
  s.now_sec=0.15; s.pivot_permission=true;
  o=f.step(s);
  check(o.phase==ForwardYawPhase::PIVOT && o.forward_speed_mps==0 &&
        o.yaw_rate_enu_radps>0 && o.yaw_rate_ned_radps<0,"positive pivot");
  s.now_sec=0.20; s.rover_yaw_enu_rad=20*pi/180; s.measured_yaw_rate_enu_radps=0;
  for(int i=0;i<4;i++) {s.now_sec+=0.05; o=f.step(s);}
  check(o.pivot_complete && o.phase==ForwardYawPhase::TRACK && o.forward_speed_mps==0,
        "settled pivot releases with zero command");
  s.now_sec+=0.05; o=f.step(s);
  check(o.phase==ForwardYawPhase::TRACK && close(o.forward_speed_mps,0.6),"next tick forward");
  s.now_sec+=0.05; s.requested_forward_speed_mps=-0.001;
  o=f.step(s);
  check(o.safe_hold && o.phase==ForwardYawPhase::SAFE_HOLD,"negative speed fails closed");
  s.now_sec+=0.05; s.requested_forward_speed_mps=0.5;
  o=f.step(s); check(o.safe_hold && close(o.forward_speed_mps,0),"fault latches");
  f.reset(); s.now_sec=10.0; s.telemetry_fresh=false;
  o=f.step(s); check(o.safe_hold && close(o.yaw_rate_ned_radps,0),"stale fails closed");
  f.reset(); s.now_sec=11.0; s.telemetry_fresh=true;
  s.rover_yaw_enu_rad=179*pi/180; s.target_yaw_enu_rad=-179*pi/180;
  o=f.step(s); check(o.phase==ForwardYawPhase::TRACK && o.yaw_rate_enu_radps>0,
                     "wrap sign");
  f.reset(); s.now_sec=12.; s.rover_yaw_enu_rad=0; s.target_yaw_enu_rad=-pi/2;
  s.pivot_permission=true; o=f.step(s);
  check(o.phase==ForwardYawPhase::PIVOT && o.yaw_rate_enu_radps<0,"negative pivot");
  s.now_sec=12.1; s.target_yaw_enu_rad=-pi/3;
  o=f.step(s); check(o.safe_hold,"pivot goal switch fails closed");
  bool threw=false;
  try {ForwardYawConfig c; c.moving_yaw_rate_max_radps=2.0; ForwardYawRatePlanner v(c);}
  catch(const std::invalid_argument&){threw=true;}
  check(threw,"unsafe yaw config rejected");
  return 0;
}
