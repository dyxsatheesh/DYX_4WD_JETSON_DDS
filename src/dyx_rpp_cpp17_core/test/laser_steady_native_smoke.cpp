#include "dyx_rpp_cpp17_core/laser_marking.hpp"
#include <cassert>
#include <iostream>
using namespace dyx::rpp;
static LaserInput good(double t) {
  LaserInput i;
  i.now_sec=t;i.terminal_identity="RUN-X|P0001";
  i.mission_enabled=true;i.laser_tool_selected=true;i.controller_enabled=true;
  i.estop_released=true;i.px4_armed_offboard=true;i.px4_link_fresh=true;
  i.stationary_hold_verified=true;i.exclusive_aux6_lease=true;
  i.raw_rtk_fixed_fresh=true;i.raw_survey_within_tolerance=true;
  i.persistent_on_lock_recorded=true;i.current_goal_live=true;
  return i;
}
int main(){
  NativeLaserMarking laser;
  assert(!laser.establish_safe_off(true,false,true));
  assert(laser.establish_safe_off(true,true,true));
  auto i=good(1.0);i.requested_mode=LaserMode::steady;
  auto o=laser.step(i);
  assert(o.state==LaserState::awaiting_on_ack && o.command==LaserCommand::on);
  assert(o.command_sequence==1 && o.aux_channel==6 && o.actuator_set==2);
  assert(o.normalized_value==1.0 && !o.marking_completed);
  i=good(1.1);i.ack=LaserAck{1,true,1.1};
  o=laser.step(i);
  assert(o.state==LaserState::steady_on && o.event==LaserEvent::on_acknowledged);
  assert(!o.marking_completed && o.command==LaserCommand::none);
  // Handoff to mission-held steady laser within 1.5 seconds.
  i=good(1.8);i.hold_active=true;i.hold_identity="RUN-X|P0001";
  i.hold_mode=LaserMode::steady;
  o=laser.step(i);
  assert(o.state==LaserState::steady_on);
  i.now_sec=120;
  o=laser.step(i);
  assert(o.state==LaserState::steady_on && o.command==LaserCommand::none);
  // Operator NEXT clears hold: OFF, not any AUX5 command.
  i=good(121);i.current_goal_live=false;
  o=laser.step(i);
  assert(o.state==LaserState::awaiting_off_ack && o.command==LaserCommand::off);
  assert(o.command_sequence==2 && o.normalized_value==-1.0);
  i=good(121.1);i.current_goal_live=false;i.ack=LaserAck{2,true,121.1};
  o=laser.step(i);
  assert(o.off_command_acknowledged && o.state==LaserState::wait_goal_clear);
  assert(!o.marking_completed && !o.on_command_may_have_been_accepted);
  // Old identity can never immediately restart.
  i=good(121.2);i.requested_mode=LaserMode::steady;
  o=laser.step(i);
  assert(o.command==LaserCommand::none);
  i=good(122);i.terminal_identity="RUN-X|P0002";i.current_goal_live=false;
  o=laser.step(i);
  assert(o.state==LaserState::idle);
  i.current_goal_live=true;i.requested_mode=LaserMode::steady;
  o=laser.step(i);
  assert(o.command==LaserCommand::none); // independent OFF proof needed
  std::cout<<"laser steady+handoff+safe off PASS\n";
}
