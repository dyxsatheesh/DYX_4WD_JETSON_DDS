#include "dds_mission_runtime_fixture.hpp"
#include <stdexcept>

int main() {
  NativeDdsMissionRuntime r;
  load_two_markers(r);
  feed_good(r,0.0);
  r.step(0.0,true,true,false,simple_mission_frame(0.0));
  assert(r.start(0.1));
  feed_good(r,0.2);
  assert(r.step(0.2,true,true,false,simple_mission_frame(0.2)).preview_forward_permitted);
  // A different source appearing under the GPS topic must not silently
  // become the surveyed rear master GNSS marking point.
  assert(r.observe_gps({fake_px4_stamp(0.3),0.3,0x5678,12.5,77.5,6,false,0})==
         DdsTelemetryDecision::invalid_data);
  assert(r.telemetry_fault_latched());
  assert(!r.has_active_mission());
  assert(r.mission().stage()==MissionStage::stopped);
  assert(!r.start(0.31));
  const auto fault=r.step(0.31,true,true,false,simple_mission_frame(0.31));
  assert(fault.telemetry_fault_latched);
  assert(!fault.preview_forward_permitted && !fault.physical_command_emitted);
  bool threw=false;
  try {load_two_markers(r);} catch(const std::logic_error &) {threw=true;}
  assert(threw);
  assert(!r.begin_new_disarmed_px4_epoch(false,true));
  assert(!r.begin_new_disarmed_px4_epoch(true,false));
  assert(r.begin_new_disarmed_px4_epoch(true,true));
  assert(!r.telemetry_fault_latched());
  assert(!r.has_active_mission());
  assert(!r.start(0.32)); // no stale replay after epoch reset
  load_two_markers(r); // reload is possible, but requires fresh safety inputs
  assert(!r.start(0.33));
}
