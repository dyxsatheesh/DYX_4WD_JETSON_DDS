#include "dyx_rpp_cpp17_core/terminal_stop.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace dyx::rpp;
namespace {
double num(std::istringstream &in) {
  std::string token;
  if (!(in >> token)) throw std::invalid_argument("missing numeric value");
  std::size_t used=0;
  const double value=std::stod(token, &used);
  if (used!=token.size()) throw std::invalid_argument("invalid numeric value");
  return value;
}
int flag(std::istringstream &in) {
  const double n=num(in);
  if(n!=0.0 && n!=1.0)throw std::invalid_argument("flag must be 0 or 1");
  return static_cast<int>(n);
}
void print(const RadialStopOutput &o) {
  std::cout << to_string(o.previous_state) << ' ' << to_string(o.state) << ' '
    << to_string(o.motion_direction) << ' ' << o.forward_speed_command_mps << ' '
    << static_cast<int>(o.hold_zero) << ' ' << o.effective_braking_speed_mps
    << ' ' << o.stop_distance_m << ' ' << o.profile_speed_mps << ' '
    << o.radial_error_m << ' ' << static_cast<int>(o.stationary) << ' '
    << o.stationary_window_sec << ' ' << to_string(o.failure) << ' '
    << static_cast<int>(o.certificate.has_value());
  if (o.certificate) {
    const auto &c=*o.certificate;
    std::cout << ' ' << c.version << ' ' << c.terminal_identity << ' '
      << c.certified_timestamp_sec << ' ' << c.radial_error_m << ' '
      << c.along_error_m << ' ' << c.cross_error_m << ' '
      << c.final_position_x_m << ' ' << c.final_position_y_m << ' '
      << c.stationary_window_sec << ' ' << c.maximum_stationary_displacement_m
      << ' ' << c.maximum_abs_yaw_rate_radps << ' '
      << c.speed_source << ' ' << c.truth_frame;
  }
  std::cout << '\n';
}
}
int main() {
  std::cout << std::setprecision(17);
  TerminalStopRegulator control;
  std::string line;
  try {
    while (std::getline(std::cin,line)) {
      if (line.empty()) continue;
      std::istringstream in(line);
      std::string op;
      in >> op;
      if (op == "reset") {control.reset();continue;}
      if (op == "cancel") {control.cancel();continue;}
      if (op != "step") throw std::invalid_argument("unknown operation");
      RadialStopInput s;
      s.monotonic_time_sec=num(in);
      s.position_sample_time_sec=num(in);
      s.active=flag(in)!=0;
      std::string identity;
      if(!(in>>identity))throw std::invalid_argument("missing terminal identity");
      if(identity!="_")s.terminal_identity=identity;
      s.along_remaining_m=num(in);
      s.cross_error_m=num(in);
      s.position_x_m=num(in);
      s.position_y_m=num(in);
      s.position_derived_speed_mps=num(in);
      s.measured_yaw_rate_radps=num(in);
      s.tracking_speed_command_mps=num(in);
      s.telemetry_fresh=flag(in)!=0;
      std::string extra;
      if(in>>extra)throw std::invalid_argument("extra input");
      print(control.step(s));
    }
  } catch(const std::exception &e) {
    std::cerr << "terminal-stop probe error: " << e.what() << '\n';
    return 2;
  }
}
