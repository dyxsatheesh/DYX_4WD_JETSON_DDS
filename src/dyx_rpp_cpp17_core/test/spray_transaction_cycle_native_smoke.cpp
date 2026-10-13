#include "dyx_rpp_cpp17_core/spray_transaction.hpp"
#include <cassert>
#include <iostream>
using namespace dyx::rpp;
static SprayInput good(double t,const char* id="RUN:alpha|GOAL:P0001") {
  SprayInput s; s.now_sec=t;s.terminal_identity=id;s.mission_enabled=true;
  s.mission_spray_hold=true;s.safety_motion_hold_verified=true;
  s.raw_rtk_fixed_survey_pass=true;s.px4_armed_offboard=true;
  s.physical_estop_released=true;s.controller_enabled=true;
  s.persistent_press_lock_recorded=true;return s;
}
int main(){
  NativeSprayTransaction tx;
  auto s=good(0);s.request_spray=true;
  assert(tx.step(s).command==SprayCommand::none); // no confirmed safe OFF
  assert(tx.establish_off_state(true,true,true));
  auto press=tx.step(s);
  assert(press.command==SprayCommand::press && press.requested_pwm_us==1400);
  assert(press.state==SprayState::waiting_press_ack);
  s=good(0.1);s.command_ack=SprayAck{press.command_sequence,true,0.1};
  assert(tx.step(s).state==SprayState::holding);
  assert(tx.step(good(3.05)).command==SprayCommand::none);
  auto rel=tx.step(good(3.11));
  assert(rel.command==SprayCommand::release && rel.requested_pwm_us==1200);
  assert(rel.completion==SprayCompletion::none);
  s=good(3.2);s.command_ack=SprayAck{rel.command_sequence,true,3.2};
  auto completed=tx.step(s);
  assert(completed.completion==SprayCompletion::completed);
  assert(completed.release_acknowledged && completed.completed_identity==s.terminal_identity);
  assert(completed.state==SprayState::wait_goal_clear && tx.off_verified());
  s=good(3.3);s.request_spray=true;
  assert(tx.step(s).command==SprayCommand::none); // no repeated same-point press
  assert(tx.identity_used(s.terminal_identity));
  s=good(3.4,"RUN:alpha|GOAL:P0002");tx.step(s);
  assert(tx.state()==SprayState::idle);
  s=good(3.5,"RUN:alpha|GOAL:P0002");s.request_spray=true;
  assert(tx.step(s).command==SprayCommand::press);
  std::cout<<"Spray cycle requires PRESS ACK, 3s hold, RELEASE ACK; no duplicate";
}
