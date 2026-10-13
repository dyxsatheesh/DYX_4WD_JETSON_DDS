#include "dyx_rpp_cpp17_core/manual_spray.hpp"
#include <cassert>
#include <stdexcept>
using namespace dyx::rpp;
static ManualSprayInput sample(double time){
 ManualSprayInput s{};s.now_sec=time;s.manual_on_request=true;
 s.request_id="id-1";s.journal_committed_request_id="id-1";
 s.authorized_operator=true;s.controller_enabled=true;
 s.px4_connected=true;s.px4_armed=true;s.px4_age_sec=0.01;
 s.physical_estop_released=true;s.mission_idle=true;s.exclusive_aux5_lease=true;
 return s;
}
int main(){
 ManualSprayConfig bad{};bad.hard_press_limit_sec=3.0;
 bool threw=false;try {NativeManualSpray x(bad);} catch (const std::invalid_argument&){threw=true;}
 assert(threw);
 NativeManualSpray m;
 assert(m.establish_off_state(true,true,true));
 auto i=sample(0.0);i.journal_committed_request_id="wrong";
 assert(m.step(i).command==SprayCommand::none);
 i=sample(0.1);i.mission_idle=false;
 assert(m.step(i).command==SprayCommand::none);
 i=sample(0.15);i.exclusive_aux5_lease=false;
 assert(m.step(i).command==SprayCommand::none);
 i=sample(0.2);i.px4_age_sec=1.0;
 assert(m.step(i).command==SprayCommand::none);
 i=sample(0.3);i.px4_armed=false;
 assert(m.step(i).command==SprayCommand::none);
 i=sample(0.4);i.physical_estop_released=false;
 assert(m.step(i).command==SprayCommand::none);
 auto press=m.step(sample(0.5));
 assert(press.command==SprayCommand::press);
 i=sample(0.6);i.mission_idle=false;
 auto released=m.step(i);
 assert(released.command==SprayCommand::release);
 assert(!released.manual_success);
 assert(m.fault_latched());
 i.now_sec=0.65;i.command_ack=SprayAck{released.command_sequence,true,0.65};
 auto result=m.step(i);
 assert(result.release_acknowledged && result.manual_failed);
 assert(m.state()==ManualSprayState::fault);
 assert(!m.reset_fault(true,false,true));
 assert(!m.reset_fault(false,true,true));
 assert(m.reset_fault(true,true,true));
 i=sample(0.7);
 assert(m.step(i).command==SprayCommand::none); // never reuse same ID
 i.request_id="id-2";i.journal_committed_request_id="id-2";
 auto new_press=m.step(i);
 assert(new_press.command==SprayCommand::press);
 i.now_sec=0.8;i.manual_off_request=true;
 auto manual_off=m.step(i);
 assert(manual_off.command==SprayCommand::release);
 i.now_sec=0.9;i.command_ack=SprayAck{manual_off.command_sequence,true,0.9};
 auto cancelled=m.step(i);
 assert(cancelled.release_acknowledged && cancelled.manual_cancelled);
 assert(!cancelled.manual_success);

 // A safety/ownership loss AFTER RELEASE is sent must invalidate success,
 // even if the old RELEASE command eventually receives an accepted ACK.
 NativeManualSpray lease_loss;
 assert(lease_loss.establish_off_state(true,true,true));
 auto new_press2=lease_loss.step(sample(10.0));
 i=sample(10.1);
 i.command_ack=SprayAck{new_press2.command_sequence,true,10.1};
 assert(lease_loss.step(i).state==ManualSprayState::holding);
 i.now_sec=13.1;i.command_ack.reset();
 auto release2=lease_loss.step(i);
 assert(release2.command==SprayCommand::release);
 i.now_sec=13.2;i.exclusive_aux5_lease=false;
 i.command_ack=SprayAck{release2.command_sequence,true,13.2};
 auto result2=lease_loss.step(i);
 assert(result2.release_acknowledged && result2.manual_failed);
 assert(!result2.manual_success && lease_loss.fault_latched());
 return 0;
}
