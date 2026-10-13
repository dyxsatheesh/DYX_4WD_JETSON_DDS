#include "dyx_rpp_cpp17_core/forward_yaw_rate.hpp"
#include <iomanip>
#include <iostream>
#include <string>

int main() {
  dyx::rpp::ForwardYawRatePlanner planner;
  std::cout << std::setprecision(17);
  std::string cmd;
  while(std::cin >> cmd) {
    if (cmd == "reset") {planner.reset(); continue;}
    if (cmd != "step") return 2;
    dyx::rpp::ForwardYawInput s;
    int enable=0,fresh=0,permission=0,stop=0;
    if(!(std::cin >> s.now_sec >> s.dt_sec >> s.rover_yaw_enu_rad >>
      s.target_yaw_enu_rad >> s.measured_yaw_rate_enu_radps >>
      s.requested_forward_speed_mps >> enable >> fresh >> permission >> stop)) return 3;
    s.motion_enabled=(enable!=0);
    s.telemetry_fresh=(fresh!=0);
    s.pivot_permission=(permission!=0);
    s.stop_requested=(stop!=0);
    const auto out=planner.step(s);
    std::cout << dyx::rpp::phase_name(out.phase) << " " << out.reason << " "
      << out.forward_speed_mps << " " << out.velocity_north_mps << " "
      << out.velocity_east_mps << " " << out.yaw_rate_enu_radps << " "
      << out.yaw_rate_ned_radps << " " << out.heading_error_rad << " "
      << int(out.safe_hold) << " " << int(out.pivot_complete) << "\n";
  }
}
