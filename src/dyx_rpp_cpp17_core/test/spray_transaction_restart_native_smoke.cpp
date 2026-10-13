#include "dyx_rpp_cpp17_core/spray_transaction.hpp"
#include <cassert>
#include <iostream>
using namespace dyx::rpp;
int main(){
 NativeSprayTransaction x;
 x.restore_uncertain("RUN:Q|P0001");
 SprayInput s;s.now_sec=10;s.terminal_identity="RUN:Q|P0001";
 auto a=x.step(s);
 assert(a.command==SprayCommand::release && a.requested_pwm_us==1200);
 assert(!x.establish_off_state(true,true,true));
 s.now_sec=10.3;
 auto b=x.step(s);
 assert(b.command==SprayCommand::release && b.command_sequence>a.command_sequence);
 s.now_sec=10.4;s.command_ack=SprayAck{a.command_sequence,true,10.4};
 assert(x.step(s).completion==SprayCompletion::none); // stale OFF acknowledgement
 s.now_sec=10.5;s.command_ack=SprayAck{b.command_sequence,true,10.5};
 auto good=x.step(s);
 assert(good.release_acknowledged && good.completion==SprayCompletion::failed);
 assert(good.state==SprayState::fault && x.off_verified());
 assert(x.reset_fault(true,true,true));
 // An uncertain transaction must remain permanently replay blocked.
 assert(x.identity_used("RUN:Q|P0001"));
 s.now_sec=11;s.command_ack.reset();s.request_spray=true;
 s.mission_enabled=s.mission_spray_hold=s.safety_motion_hold_verified=true;
 s.raw_rtk_fixed_survey_pass=s.px4_armed_offboard=true;
 s.physical_estop_released=s.controller_enabled=s.persistent_press_lock_recorded=true;
 assert(x.step(s).command==SprayCommand::none);
 std::cout<<"Restart uncertainty requires OFF ACK; never replays uncertain press";
}
