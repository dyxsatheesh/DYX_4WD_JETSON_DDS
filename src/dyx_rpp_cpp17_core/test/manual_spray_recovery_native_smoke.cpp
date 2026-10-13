#include "dyx_rpp_cpp17_core/manual_spray.hpp"
#include <cassert>
using namespace dyx::rpp;
static ManualSprayInput ready(double now,const char *id="manual-99"){
 ManualSprayInput s{};s.now_sec=now;s.manual_on_request=true;
 s.request_id=id;s.journal_committed_request_id=id;
 s.authorized_operator=true;s.controller_enabled=true;
 s.px4_connected=true;s.px4_armed=true;s.px4_age_sec=0.02;
 s.physical_estop_released=true;s.mission_idle=true;s.exclusive_aux5_lease=true;return s;
}
int main(){
 NativeManualSpray x;
 assert(x.establish_off_state(true,true,true));
 auto press=x.step(ready(0.1));
 assert(press.command==SprayCommand::press);
 auto i=ready(1.0); // unanswered PRESS times out; never re-press
 auto release=x.step(i);
 assert(release.command==SprayCommand::release && x.fault_latched());
 i.now_sec=1.4;
 i.command_ack=SprayAck{press.command_sequence,true,1.39}; // stale PRESS accepted: ignore
 auto retry=x.step(i);
 assert(retry.command==SprayCommand::release);
 assert(retry.command_sequence!=release.command_sequence);
 i.now_sec=1.45;i.command_ack=SprayAck{retry.command_sequence,true,1.45};
 auto f=x.step(i);
 assert(f.release_acknowledged && f.manual_failed);
 assert(!f.manual_success);

 NativeManualSpray after_restart;
 after_restart.restore_uncertain("crashed-press");
 i=ready(10.0,"crashed-press");
 auto recovery=after_restart.step(i);
 assert(recovery.command==SprayCommand::release);
 assert(recovery.state==ManualSprayState::recovering_release);
 assert(after_restart.request_used("crashed-press"));
 i.command_ack=SprayAck{recovery.command_sequence,true,10.1};i.now_sec=10.1;
 auto recovered=after_restart.step(i);
 assert(recovered.release_acknowledged && recovered.manual_failed);
 assert(!after_restart.reset_fault(true,false,true));
 assert(after_restart.reset_fault(true,true,true));
 assert(after_restart.step(ready(10.2,"crashed-press")).command==SprayCommand::none);

 NativeManualSpray watchdog;
 assert(watchdog.establish_off_state(true,true,true));
 auto start=watchdog.step(ready(0.1,"watchdog"));
 i=ready(0.2,"watchdog");i.command_ack=SprayAck{start.command_sequence,true,0.2};
 assert(watchdog.step(i).state==ManualSprayState::holding);
 i.now_sec=5.1;i.command_ack.reset();
 auto forced=watchdog.step(i);
 assert(forced.command==SprayCommand::release && watchdog.fault_latched());
 assert(!forced.manual_success);
 return 0;
}
