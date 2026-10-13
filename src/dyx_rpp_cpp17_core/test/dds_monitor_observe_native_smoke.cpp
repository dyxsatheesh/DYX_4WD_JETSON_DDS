#include "dyx_rpp_cpp17_core/dds_actuator_monitor.hpp"
#include <cassert>
using namespace dyx::rpp;
int main() {
  NativeDdsActuatorMonitor monitor;
  ObservedPx4Ack ack{};
  ack.command=187;
  ack.result=0;
  ack.target_system=1;
  ack.target_component=1;
  ack.from_external=true;
  ack.px4_timestamp_us=1000000;
  assert(monitor.observe(ack,10.0)==DdsObservationResult::observed);
  auto s=monitor.status(10.5);
  assert(s.observed_actuator_acks==1 && s.accepted_actuator_acks==1);
  assert(s.last_ack_fresh && s.last_ack_was_accepted);
  assert(!s.command_dispatch_permitted && !s.unique_receipt_proven);
  assert(!s.physical_off_proven && !s.actuator_success_proven);
  assert(!monitor.status(12.1).last_ack_fresh);
  ack.px4_timestamp_us=2000000;ack.result=2;
  assert(monitor.observe(ack,12.2)==DdsObservationResult::observed);
  s=monitor.status(12.2);
  assert(s.refused_actuator_acks==1 && !s.last_ack_was_accepted);
  assert(!s.command_dispatch_permitted && !s.actuator_success_proven);
}
