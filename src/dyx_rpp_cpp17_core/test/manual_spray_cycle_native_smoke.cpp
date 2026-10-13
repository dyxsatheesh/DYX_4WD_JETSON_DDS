#include "dyx_rpp_cpp17_core/manual_spray.hpp"
#include <cassert>
using namespace dyx::rpp;

static ManualSprayInput good(double now) {
  ManualSprayInput i{};
  i.now_sec=now;
  i.manual_on_request=true;
  i.request_id="ui-press-17";
  i.journal_committed_request_id=i.request_id;
  i.authorized_operator=true;
  i.controller_enabled=true;
  i.px4_connected=true;
  i.px4_armed=true;
  i.px4_age_sec=0.03;
  i.physical_estop_released=true;
  i.mission_idle=true;
  i.exclusive_aux5_lease=true;
  return i;
}
int main() {
  NativeManualSpray manual;
  auto i=good(1.0);
  auto denied=manual.step(i);
  assert(denied.command==SprayCommand::none); // power-up OFF unverified
  assert(!manual.establish_off_state(true,false,true));
  assert(manual.establish_off_state(true,true,true));
  auto press=manual.step(good(1.1));
  assert(press.command==SprayCommand::press && press.requested_pwm_us==1400);
  assert(press.state==ManualSprayState::waiting_press_ack);
  i=good(1.15);
  assert(manual.step(i).command==SprayCommand::none); // ON keepalive does not extend
  i.command_ack=SprayAck{press.command_sequence,true,1.16};
  i.now_sec=1.16;
  assert(manual.step(i).state==ManualSprayState::holding);
  i.command_ack.reset();i.now_sec=4.15;
  assert(manual.step(i).command==SprayCommand::none);
  i.now_sec=4.16;
  auto release=manual.step(i);
  assert(release.command==SprayCommand::release && release.requested_pwm_us==1200);
  assert(!release.manual_success && !release.release_acknowledged);
  i.now_sec=4.23;
  i.command_ack=SprayAck{press.command_sequence,true,4.22}; // stale PRESS ack
  assert(manual.step(i).command==SprayCommand::none);
  i.command_ack=SprayAck{release.command_sequence,true,4.25};
  i.now_sec=4.25;
  auto done=manual.step(i);
  assert(done.release_acknowledged && done.manual_success);
  assert(!done.manual_failed && !done.manual_cancelled);
  assert(done.resolved_request_id=="ui-press-17");
  assert(done.state==ManualSprayState::idle);
  assert(manual.request_used("ui-press-17"));
  assert(!manual.off_verified()); // RELEASE ACK is not physical OFF feedback
  assert(done.press_may_be_active); // unknown until independently verified
  i=good(4.26);i.request_id="ui-press-18";
  i.journal_committed_request_id=i.request_id;
  assert(manual.step(i).command==SprayCommand::none); // no second PRESS on ACK alone
  assert(!manual.establish_off_state(true,false,true)); // must be disarmed
  assert(manual.establish_off_state(true,true,true)); // independent OFF verification
  assert(manual.off_verified());
  i=good(4.27);i.command_ack.reset();
  assert(!manual.step(i).manual_success); // completion event one tick
  assert(manual.step(i).command==SprayCommand::none); // replay ID cannot PRESS
  i=good(4.30);i.request_id="ui-press-18";
  i.journal_committed_request_id=i.request_id;
  auto second=manual.step(i);
  assert(second.command==SprayCommand::press); // independently verified new cycle
  i.now_sec=4.31;i.manual_off_request=true;
  auto abort=manual.step(i);
  assert(abort.command==SprayCommand::release);
  i.now_sec=4.32;i.command_ack=SprayAck{abort.command_sequence,true,4.32};
  const auto cancelled=manual.step(i);
  assert(cancelled.release_acknowledged && cancelled.manual_cancelled);
  assert(!cancelled.manual_success);
  return 0;
}
