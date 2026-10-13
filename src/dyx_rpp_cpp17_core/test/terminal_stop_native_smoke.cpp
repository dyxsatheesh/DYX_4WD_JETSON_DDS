#include "dyx_rpp_cpp17_core/terminal_stop.hpp"
#include <cmath>
#include <stdexcept>

using namespace dyx::rpp;
void check(bool v) {if(!v)throw std::runtime_error("radial-stop smoke assertion failed");}
RadialStopInput sample(double t, double remaining=0.012, double cross=0.001,
                       double command=0.2) {
  RadialStopInput s;
  s.monotonic_time_sec=t;
  s.position_sample_time_sec=t;
  s.active=true;
  s.terminal_identity="P0001";
  s.along_remaining_m=remaining;
  s.cross_error_m=cross;
  s.position_x_m=4.0;
  s.position_y_m=1.0;
  s.position_derived_speed_mps=0.0;
  s.measured_yaw_rate_radps=0.01;
  s.tracking_speed_command_mps=command;
  s.telemetry_fresh=true;
  return s;
}
int main() {
  TerminalStopRegulator control;
  auto s=sample(0.0,2.0);
  s.active=false;
  auto out=control.step(s);
  check(out.state==RadialStopState::track && out.forward_speed_command_mps==0.2);
  out=control.step(sample(0.05));
  check(out.state==RadialStopState::zero_latch && out.hold_zero);
  for(int i=2;i<=11;i++) out=control.step(sample(i*0.05));
  check(out.state==RadialStopState::settle);
  out=control.step(sample(0.60));
  check(out.state==RadialStopState::certified && out.certificate.has_value());
  check(out.certificate->terminal_identity=="P0001");
  check(out.certificate->version==2 && out.certificate->radial_error_m<0.02);
  out=control.step(sample(0.65));
  check(out.state==RadialStopState::hold_zero && out.certificate.has_value());
  control.reset();
  s=sample(1.0,0.3);
  out=control.step(s);
  check(out.state==RadialStopState::terminal_guidance);
  s=sample(1.05,0.1,0.0,0.35);
  out=control.step(s);
  check(out.state==RadialStopState::brake_profile && out.forward_speed_command_mps>0.0);
  s=sample(1.10,0.07,0.0,0.3);
  s.terminal_identity="P0002";
  out=control.step(s);
  check(out.state==RadialStopState::hold_fail && out.failure==RadialStopFailure::identity_changed);
  control.cancel();
  s=sample(2.0);
  s.telemetry_fresh=false;
  out=control.step(s);
  check(out.state==RadialStopState::hold_fail && out.failure==RadialStopFailure::stale_telemetry);
  control.reset();
  s=sample(2.1);s.tracking_speed_command_mps=-0.1;
  out=control.step(s);
  check(out.failure==RadialStopFailure::invalid_measurement && out.hold_zero);
  control.reset();
  out=control.step(sample(3.0,1.0));
  check(out.state==RadialStopState::track);
  out=control.step(sample(18.1,1.0));
  check(out.failure==RadialStopFailure::terminal_timeout && out.hold_zero);
  return 0;
}
