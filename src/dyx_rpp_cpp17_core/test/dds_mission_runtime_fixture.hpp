#pragma once
// Purely synthetic offline safety frames; NEVER connect tests to PX4 hardware.
#include "dyx_rpp_cpp17_core/dds_mission_runtime.hpp"
#include <cassert>
#include <cstdint>
using namespace dyx::rpp;

inline std::uint64_t fake_px4_stamp(double t) {
  return static_cast<std::uint64_t>((t + 10.0)*1000000.0);
}
inline void feed_good(NativeDdsMissionRuntime &r, double t, int fix=6,
                      bool obstacle_clear=true) {
  const auto stamp=fake_px4_stamp(t);
  assert(r.observe_vehicle({stamp,t,2,14,3,false,0})==DdsTelemetryDecision::accepted);
  assert(r.observe_battery({stamp,t,24.9,true,false,8,0})==DdsTelemetryDecision::accepted);
  assert(r.observe_gps({stamp,t,0x1234,12.5,77.5,fix,false,0})==DdsTelemetryDecision::accepted);
  assert(r.observe_estimator({stamp,t,true,true,true,false,0.2,0.1,0.0})==DdsTelemetryDecision::accepted);
  assert(r.observe_corrections({t,0.2,true})==DdsTelemetryDecision::accepted);
  assert(r.observe_obstacle({t,obstacle_clear})==DdsTelemetryDecision::accepted);
}
inline MissionSequenceInput simple_mission_frame(double t) {
  MissionSequenceInput m;
  m.monotonic_time_sec=t;
  m.emergency_stop=false;
  m.measured_forward_speed_mps=0.4;
  return m;
}
inline void load_two_markers(NativeDdsMissionRuntime &r) {
  r.load({{0,0,"P0001","RUN:H4B:P0001"},
          {100,1,"P0002","RUN:H4B:P0002"}},
         MissionMode::automatic,true,true);
}
