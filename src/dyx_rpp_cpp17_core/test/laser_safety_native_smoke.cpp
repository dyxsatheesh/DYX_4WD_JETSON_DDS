#include "dyx_rpp_cpp17_core/laser_marking.hpp"
#include <cassert>
#include <iostream>
using namespace dyx::rpp;
static LaserInput good(double t) {
  LaserInput i;
  i.now_sec=t;i.terminal_identity="RUN-Z|P0002";
  i.mission_enabled=true;i.laser_tool_selected=true;i.controller_enabled=true;
  i.estop_released=true;i.px4_armed_offboard=true;i.px4_link_fresh=true;
  i.stationary_hold_verified=true;i.exclusive_aux6_lease=true;
  i.raw_rtk_fixed_fresh=true;i.raw_survey_within_tolerance=true;
  i.persistent_on_lock_recorded=true;i.current_goal_live=true;
  i.requested_mode=LaserMode::steady;
  return i;
}
int main(){
  {NativeLaserMarking x;auto i=good(1);
   assert(x.step(i).command==LaserCommand::none); // OFF unverified
   assert(x.establish_safe_off(true,true,true));
   i.persistent_on_lock_recorded=false;
   assert(x.step(i).command==LaserCommand::none);
   i.now_sec=2;i.persistent_on_lock_recorded=true;i.raw_survey_within_tolerance=false;
   assert(x.step(i).command==LaserCommand::none);
   i.now_sec=3;i.raw_survey_within_tolerance=true;i.exclusive_aux6_lease=false;
   assert(x.step(i).command==LaserCommand::none);
   i.now_sec=4;i.exclusive_aux6_lease=true;
   assert(x.step(i).command==LaserCommand::on);
   // E-stop while ON is in flight: wait for its ACK, then OFF immediately.
   i=good(4.1);i.estop_released=false;
   auto o=x.step(i);assert(o.command==LaserCommand::none);
   i.now_sec=4.15;i.ack=LaserAck{1,true,4.15};
   o=x.step(i);assert(o.command==LaserCommand::off && o.command_sequence==2);
   i.now_sec=4.2;i.ack=LaserAck{2,true,4.2};
   o=x.step(i);assert(o.state==LaserState::fault && o.off_command_acknowledged);
   assert(!x.reset_fault(true,false,true));
   assert(!x.reset_fault(true,true,false));
   assert(x.reset_fault(true,true,true));
   assert(x.identity_used("RUN-Z|P0002"));
  }
  {NativeLaserMarking x;assert(x.establish_safe_off(true,true,true));
   auto i=good(1);auto o=x.step(i);assert(o.command==LaserCommand::on);
   i=good(2.01);o=x.step(i);assert(o.command==LaserCommand::off);
   // Failed OFF command is never taken as a safe physical release.
   i=good(2.1);i.ack=LaserAck{2,false,2.1};
   o=x.step(i);assert(o.command==LaserCommand::none);
   o=x.step(good(2.4));assert(o.command==LaserCommand::off && o.command_sequence==3);
   i=good(2.5);i.ack=LaserAck{3,true,2.5};
   o=x.step(i);assert(o.state==LaserState::fault && o.off_command_acknowledged);
  }
  {NativeLaserMarking x;x.restore_uncertain("RUN-Z|P0004");
   auto i=good(1);i.terminal_identity="RUN-Z|P0004";
   auto o=x.step(i);assert(o.command==LaserCommand::off && o.command_sequence==1);
   assert(x.identity_used("RUN-Z|P0004"));
   i.now_sec=1.1;i.ack=LaserAck{1,true,1.1};
   o=x.step(i);assert(o.state==LaserState::fault && o.off_command_acknowledged);
  }
  {NativeLaserMarking x;assert(x.establish_safe_off(true,true,true));
   auto i=good(1);auto o=x.step(i);assert(o.command==LaserCommand::on);
   // Mission moves on before ON ACK arrives: no late marking event.
   i=good(1.1);i.current_goal_live=false;i.ack=LaserAck{1,true,1.1};
   o=x.step(i);
   assert(o.command==LaserCommand::off && o.event!=LaserEvent::on_acknowledged);
  }
  {NativeLaserMarking x;assert(x.establish_safe_off(true,true,true));
   auto i=good(1);auto o=x.step(i);assert(o.command==LaserCommand::on);
   i=good(1.1);i.ack=LaserAck{1,true,1.1};x.step(i);
   x.stop(); // explicit STOP must turn off, without another ON
   i=good(1.2);o=x.step(i);assert(o.command==LaserCommand::off);
   i=good(1.3);i.ack=LaserAck{2,true,1.3};o=x.step(i);
   assert(o.state==LaserState::wait_goal_clear && !x.fault_latched());
  }
  std::cout<<"laser fail-closed safety + restart PASS\n";
}
