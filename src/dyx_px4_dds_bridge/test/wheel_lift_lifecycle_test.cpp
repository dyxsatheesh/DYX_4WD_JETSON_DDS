#include "dyx_px4_dds_bridge/wheel_lift_lifecycle.hpp"
#include <cassert>
#include <limits>
using namespace dyx::bridge;
int main(){
 LifecycleInput x{true,true,true,true,true,false,true,true,false,false,false,false,false,true,0.10,0.05};
 assert(evaluate_lifecycle(x).motion);
 x.stop_requested=true; assert(!evaluate_lifecycle(x).motion&&evaluate_lifecycle(x).request_disarm);
 x.stop_requested=false; x.gpio_ok=false; assert(evaluate_lifecycle(x).stop_latch);
 x.gpio_ok=true; x.command_fresh=false; assert(evaluate_lifecycle(x).request_disarm);
 x.command_fresh=true; x.sole_writer=false; assert(!evaluate_lifecycle(x).motion);
 x.sole_writer=true; x.awaiting_ack=true; x.ack_matches=true; x.ack_accepted=false;
 assert(evaluate_lifecycle(x).stop_latch&&evaluate_lifecycle(x).accept_ack);
 x.ack_matches=false; assert(!evaluate_lifecycle(x).accept_ack);
 x.awaiting_ack=false; x.ack_timed_out=true; assert(evaluate_lifecycle(x).request_disarm);
 x.ack_timed_out=false; x.speed=.16; assert(!evaluate_lifecycle(x).motion);
 x.speed=.10; x.yaw_rate=std::numeric_limits<double>::quiet_NaN(); assert(!evaluate_lifecycle(x).motion);
 x.yaw_rate=.05; x.armed=false; assert(!evaluate_lifecycle(x).motion&&!evaluate_lifecycle(x).request_disarm);
 x.armed=true; x.offboard=false; assert(!evaluate_lifecycle(x).motion);
 x.offboard=true; x.latched=true; assert(!evaluate_lifecycle(x).motion);
}
