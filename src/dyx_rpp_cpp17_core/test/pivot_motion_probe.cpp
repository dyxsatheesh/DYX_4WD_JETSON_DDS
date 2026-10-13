#include "dyx_rpp_cpp17_core/pivot_motion.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
using namespace dyx::rpp;
int main(){
  VerifiedPivotStateMachine fsm;
  std::string line;
  std::cout<<std::setprecision(17);
  while(std::getline(std::cin,line)) {
    if(line.empty())continue;
    try{
      std::istringstream in(line);
      std::string op;in>>op;
      if(op=="reset") {
        double t; if(!(in>>t))throw std::invalid_argument("reset time");
        fsm.reset(t);continue;
      }
      if(op!="step")throw std::invalid_argument("unknown action");
      PivotMotionInput s{};
      int fresh,pivot,brake,done;
      if(!(in>>s.monotonic_time_sec>>s.dt_sec>>s.anchor_radial_error_m
          >>s.measured_linear_speed_mps>>s.measured_yaw_rate_radps>>s.heading_error_rad
          >>fresh>>pivot>>brake>>done))throw std::invalid_argument("incomplete pivot input");
      s.telemetry_fresh=fresh!=0;s.pivot_requested=pivot!=0;
      s.brake_to_anchor_requested=brake!=0;s.recapture_complete=done!=0;
      const auto r=fsm.step(s);
      const auto &a=r.stop_certificate;const auto &b=r.release_certificate;
      std::cout<<motion_name(r.state)<<' '<<motion_name(r.previous_state)<<' '
        <<motion_name(r.directive)<<' '<<(r.transition_reason.empty()?"-":r.transition_reason)<<' '
        <<int(a.position_ok)<<' '<<int(a.linear_speed_ok)<<' '<<int(a.yaw_rate_ok)<<' '
        <<int(a.telemetry_fresh)<<' '<<a.held_sec<<' '<<a.required_sec<<' '<<int(a.valid)<<' '
        <<int(b.position_ok)<<' '<<int(b.heading_ok)<<' '<<int(b.linear_speed_ok)<<' '
        <<int(b.yaw_rate_ok)<<' '<<int(b.telemetry_fresh)<<' '<<b.held_sec<<' '
        <<b.required_sec<<' '<<int(b.valid)<<' '<<r.max_pivot_drift_m<<' '
        <<r.recenter_attempts<<' '<<r.bounded_dt_sec<<' '<<r.state_elapsed_sec<<' '
        <<int(r.failed)<<'\n';
    }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 2;}
  }
}
