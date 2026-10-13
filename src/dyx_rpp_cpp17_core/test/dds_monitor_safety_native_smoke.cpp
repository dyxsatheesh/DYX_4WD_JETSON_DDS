#include "dyx_rpp_cpp17_core/dds_actuator_monitor.hpp"
#include <cassert>
#include <limits>
using namespace dyx::rpp;
int main(){
  NativeDdsActuatorMonitor monitor;
  ObservedPx4Ack ack{};
  ack.command=187;ack.target_system=1;ack.target_component=1;
  assert(monitor.observe(ack,1.0)==DdsObservationResult::invalid_payload);
  ack.px4_timestamp_us=100;ack.result=0;
  assert(monitor.observe(ack,2.0)==DdsObservationResult::observed);
  assert(monitor.observe(ack,2.1)==DdsObservationResult::duplicate_or_reordered);
  ack.px4_timestamp_us=90;
  assert(monitor.observe(ack,2.2)==DdsObservationResult::duplicate_or_reordered);
  ack.px4_timestamp_us=110;ack.result=10;
  assert(monitor.observe(ack,2.3)==DdsObservationResult::invalid_payload);
  ack.result=0;ack.command=100;
  assert(monitor.observe(ack,2.4)==DdsObservationResult::unrelated);
  auto s=monitor.status(2.4);
  assert(s.accepted_actuator_acks==1 && s.rejected_observations==4);
  assert(s.unrelated_acks==1 && !s.actuator_success_proven);
  assert(monitor.observe(ack,std::numeric_limits<double>::quiet_NaN())==
         DdsObservationResult::invalid_clock);
  s=monitor.status(2.4);
  assert(s.integrity_fault_latched && !s.last_ack_fresh);
  assert(!s.command_dispatch_permitted && !s.unique_receipt_proven);
  assert(!s.physical_off_proven);
}
