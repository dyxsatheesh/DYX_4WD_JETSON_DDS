#include "dyx_rpp_cpp17_core/spray_transaction.hpp"
#include <cassert>
#include <iostream>
using namespace dyx::rpp;
static SprayInput ok(double t){SprayInput s;s.now_sec=t;s.terminal_identity="RUN:B|P0001";
 s.mission_enabled=s.mission_spray_hold=s.safety_motion_hold_verified=true;
 s.raw_rtk_fixed_survey_pass=s.px4_armed_offboard=true;
 s.physical_estop_released=s.controller_enabled=s.persistent_press_lock_recorded=true;
 return s;}
int main(){
 NativeSprayTransaction x;assert(x.establish_off_state(true,true,true));
 auto s=ok(0);s.request_spray=true;auto p=x.step(s);assert(p.command==SprayCommand::press);
 auto r=x.step(ok(1.1));assert(r.command==SprayCommand::release);
 assert(x.fault_latched() && r.state==SprayState::recovering_release);
 assert(x.step(ok(1.2)).completion==SprayCompletion::none);
 s=ok(1.4);s.command_ack=SprayAck{p.command_sequence,true,1.39};
 auto retry=x.step(s); assert(retry.command==SprayCommand::release); // stale PRESS ack not accepted
 s=ok(1.5);s.command_ack=SprayAck{retry.command_sequence,true,1.5};
 auto f=x.step(s);assert(f.release_acknowledged && f.completion==SprayCompletion::failed);
 assert(f.state==SprayState::fault);
 assert(!x.reset_fault(false,true,true));
 assert(x.reset_fault(true,true,true));
 s=ok(1.6);s.request_spray=true;
 assert(x.step(s).command==SprayCommand::none); // no replay of same marking

 NativeSprayTransaction estop;assert(estop.establish_off_state(true,true,true));
 s=ok(2);s.request_spray=true;p=estop.step(s);
 s=ok(2.1);s.command_ack=SprayAck{p.command_sequence,true,2.1};
 assert(estop.step(s).state==SprayState::holding);
 s=ok(2.2);s.physical_estop_released=false;
 r=estop.step(s);assert(r.command==SprayCommand::release);
 s=ok(2.3);s.command_ack=SprayAck{r.command_sequence,true,2.3};
 f=estop.step(s); assert(f.release_acknowledged && f.completion==SprayCompletion::failed);
 assert(f.state==SprayState::fault);
 // Operator STOP intentionally aborts without declaring a successful marking.
 NativeSprayTransaction stopped;assert(stopped.establish_off_state(true,true,true));
 s=ok(3.0);s.request_spray=true;p=stopped.step(s);
 s=ok(3.1);s.command_ack=SprayAck{p.command_sequence,true,3.1};
 assert(stopped.step(s).state==SprayState::holding);
 stopped.stop();
 r=stopped.step(ok(3.2));assert(r.command==SprayCommand::release);
 s=ok(3.3);s.command_ack=SprayAck{r.command_sequence,true,3.3};
 f=stopped.step(s);assert(f.completion==SprayCompletion::cancelled);
 assert(f.release_acknowledged && f.state==SprayState::wait_goal_clear);
 assert(!stopped.fault_latched());
 std::cout<<"PRESS timeout and E-stop cause RELEASE + fault, no unsafe retry";
}
