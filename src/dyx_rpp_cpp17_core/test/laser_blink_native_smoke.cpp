#include "dyx_rpp_cpp17_core/laser_marking.hpp"
#include <cassert>
#include <iostream>
using namespace dyx::rpp;
static LaserInput blink(double t) {
  LaserInput i;
  i.now_sec=t;i.terminal_identity="RUN-Y|P0003";
  i.mission_enabled=true;i.laser_tool_selected=true;i.controller_enabled=true;
  i.estop_released=true;i.px4_armed_offboard=true;i.px4_link_fresh=true;
  i.stationary_hold_verified=true;i.exclusive_aux6_lease=true;
  i.raw_rtk_fixed_fresh=true;i.raw_survey_outside_tolerance=true;
  i.persistent_on_lock_recorded=true;i.current_goal_live=true;
  i.hold_active=true;i.hold_identity=i.terminal_identity;i.hold_mode=LaserMode::blink;
  return i;
}
int main(){
  NativeLaserMarking laser;assert(laser.establish_safe_off(true,true,true));
  auto i=blink(5.0);i.requested_mode=LaserMode::blink;
  auto o=laser.step(i);
  assert(o.command==LaserCommand::on && o.command_sequence==1);
  i=blink(5.1);i.ack=LaserAck{1,true,5.1};
  o=laser.step(i);
  assert(o.state==LaserState::blink_on && o.event==LaserEvent::none && !o.marking_completed);
  o=laser.step(blink(5.59));
  assert(o.command==LaserCommand::none);
  o=laser.step(blink(5.61));
  assert(o.command==LaserCommand::off && o.command_sequence==2);
  // No overlapping ON while OFF acknowledgement is in flight.
  o=laser.step(blink(5.65));
  assert(o.command==LaserCommand::none);
  i=blink(5.70);i.ack=LaserAck{2,true,5.70};
  o=laser.step(i);
  assert(o.state==LaserState::blink_off && o.off_command_acknowledged);
  o=laser.step(blink(6.19));
  assert(o.command==LaserCommand::none);
  o=laser.step(blink(6.21));
  assert(o.command==LaserCommand::on && o.command_sequence==3);
  i=blink(6.25);i.ack=LaserAck{2,true,6.25};
  o=laser.step(i);
  assert(o.command==LaserCommand::none && o.state==LaserState::awaiting_on_ack);
  i=blink(6.30);i.ack=LaserAck{3,true,6.30};
  o=laser.step(i);
  assert(o.state==LaserState::blink_on && !o.marking_completed);
  // NEXT removes the blink hold: immediate OFF intent.
  i=blink(6.4);i.hold_active=false;
  o=laser.step(i);
  assert(o.command==LaserCommand::off && o.command_sequence==4);
  i=blink(6.5);i.hold_active=false;i.ack=LaserAck{4,true,6.5};
  o=laser.step(i);
  assert(o.off_command_acknowledged && o.state==LaserState::wait_goal_clear);
  assert(!o.marking_completed);
  std::cout<<"laser blink ON/OFF ack sequencing PASS\n";
}
