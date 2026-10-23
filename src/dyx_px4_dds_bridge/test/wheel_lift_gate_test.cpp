#include "dyx_px4_dds_bridge/wheel_lift_gate.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
 using namespace dyx::bridge;
 WheelLiftEvidence e{};
 assert(!inspect_wheel_lift(e).publish_authorized);
 e.operator_present=true;
 e.mechanical_wheels_lifted_confirmed=true;
 e.physical_estop_released_verified=true;
 e.estop_signal_fresh=true;
 e.estop_asserted=false;
 e.rc_takeover_verified=true;
 e.px4_armed=true;
 e.offboard_active=true;
 e.px4_status_fresh=true;
 e.command_fresh=true;
 e.heartbeat_fresh=true;
 e.writer_graph_quiet=true;
 e.output_path_certified=true;
 e.speed_mps=0.15;
 e.yaw_rate_radps=0.10;
 assert(inspect_wheel_lift(e).prerequisites_met);
 assert(!inspect_wheel_lift(e).publish_authorized);
 auto check=[&](auto mutate){auto c=e;mutate(c);assert(!inspect_wheel_lift(c).prerequisites_met);assert(!inspect_wheel_lift(c).publish_authorized);};
 check([](auto& c){c.operator_present=false;});
 check([](auto& c){c.mechanical_wheels_lifted_confirmed=false;});
 check([](auto& c){c.physical_estop_released_verified=false;});
 check([](auto& c){c.estop_signal_fresh=false;});
 check([](auto& c){c.estop_asserted=true;});
 check([](auto& c){c.rc_takeover_verified=false;});
 check([](auto& c){c.px4_armed=false;});
 check([](auto& c){c.offboard_active=false;});
 check([](auto& c){c.px4_status_fresh=false;});
 check([](auto& c){c.command_fresh=false;});
 check([](auto& c){c.heartbeat_fresh=false;});
 check([](auto& c){c.writer_graph_quiet=false;});
 check([](auto& c){c.speed_mps=0.151;});
 check([](auto& c){c.yaw_rate_radps=-0.101;});
 check([](auto& c){c.speed_mps=std::numeric_limits<double>::quiet_NaN();});
 check([](auto& c){c.yaw_rate_radps=std::numeric_limits<double>::infinity();});
 check([](auto& c){c.output_path_certified=false;});
 std::cout << "PASS: wheel lift preview gate, 18 cases; publishing always denied\n";
}
