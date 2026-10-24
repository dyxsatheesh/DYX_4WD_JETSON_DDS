#include "dyx_px4_dds_bridge/wheel_lift_stop_policy.hpp"
#include <cassert>
using dyx::bridge::WheelLiftStopInputs;
using dyx::bridge::wheel_lift_stop_policy;
int main() {
 WheelLiftStopInputs e{true,true,true,true,true,true,true,true};
 auto d=wheel_lift_stop_policy(e); assert(!d.latch && !d.request_disarm);
 e.gpio_ok=false; d=wheel_lift_stop_policy(e); assert(d.latch && d.request_disarm);
 e.gpio_ok=true; e.px4_ok=false; d=wheel_lift_stop_policy(e); assert(d.latch && d.request_disarm);
 e.px4_ok=true; e.sole_writer=false; d=wheel_lift_stop_policy(e); assert(d.latch && d.request_disarm);
 e.sole_writer=true; e.command_fresh=false; d=wheel_lift_stop_policy(e); assert(d.latch && d.request_disarm);
 e.command_fresh=true; e.armed=false; e.gpio_ok=false; d=wheel_lift_stop_policy(e); assert(d.latch && !d.request_disarm);
 e.had_prestream=false; d=wheel_lift_stop_policy(e); assert(!d.latch && !d.request_disarm);
 e.gpio_ok=true; e.command_fresh=false; e.was_offboard=false; d=wheel_lift_stop_policy(e); assert(!d.latch && !d.request_disarm);
 e.was_offboard=true; e.command_valid=false; d=wheel_lift_stop_policy(e); assert(!d.latch && !d.request_disarm);
}
