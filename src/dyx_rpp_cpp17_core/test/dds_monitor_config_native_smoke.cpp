#include "dyx_rpp_cpp17_core/dds_actuator_monitor.hpp"
#include <cassert>
#include <limits>
#include <stdexcept>
using namespace dyx::rpp;
int main(){
  bool threw=false;
  try { NativeDdsActuatorMonitor m(DdsMonitorConfig{0.0,187});(void)m; }
  catch(const std::invalid_argument&){threw=true;}
  assert(threw);
  threw=false;
  try {NativeDdsActuatorMonitor m(DdsMonitorConfig{1.0,400});(void)m;}
  catch(const std::invalid_argument&){threw=true;}
  assert(threw);
  NativeDdsActuatorMonitor m(DdsMonitorConfig{0.5,187});
  auto s=m.status(1.0);
  assert(!s.last_ack_fresh && !s.command_dispatch_permitted);
  ObservedPx4Ack a{};a.command=187;a.result=0;a.target_system=1;
  a.target_component=1;a.px4_timestamp_us=100;
  assert(m.observe(a,4.0)==DdsObservationResult::observed);
  assert(m.status(4.5).last_ack_fresh);
  assert(!m.status(4.5001).last_ack_fresh);
  assert(m.observe(a,3.0)==DdsObservationResult::invalid_clock);
  assert(m.status(4.0).integrity_fault_latched);
  assert(!m.status(4.0).command_dispatch_permitted);
}
