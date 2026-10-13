#pragma once
// Synthetic, hardware-free data for 2H4F regression tests.
#include "dyx_rpp_cpp17_core/backend_live_status.hpp"
#include <cassert>
#include <cstdint>
#include <string>
using namespace dyx::rpp;

inline void backend_live_feed(NativeMissionCommandGateway& g, double t,
                              int fix = 6, bool obstacle_clear = true) {
  const auto stamp = static_cast<std::uint64_t>((t + 10.0)*1000000.0);
  assert(g.observe_vehicle({stamp,t,2,14,3,false,0})==DdsTelemetryDecision::accepted);
  assert(g.observe_battery({stamp,t,25.2,true,false,8,0})==DdsTelemetryDecision::accepted);
  assert(g.observe_gps({stamp,t,0x1234,12.5,77.5,fix,false,0})==DdsTelemetryDecision::accepted);
  assert(g.observe_estimator({stamp,t,true,true,true,false,0.2,0.1,0.0})==DdsTelemetryDecision::accepted);
  assert(g.observe_corrections({t,0.2,true})==DdsTelemetryDecision::accepted);
  assert(g.observe_obstacle({t,obstacle_clear})==DdsTelemetryDecision::accepted);
}
inline DdsMissionRuntimeOutput backend_live_tick(NativeMissionCommandGateway& g, double t) {
  MissionSequenceInput in;
  in.monotonic_time_sec=t;
  in.emergency_stop=false;
  in.measured_forward_speed_mps=0.3;
  return g.tick(t,true,true,false,in);
}
inline BackendLiveStatusRequest backend_live_req(double t) {
  BackendLiveStatusRequest r;
  r.principal={true,true,false,"SESSION_LIVE_READ_ONLY"};
  r.host_now_sec=t;
  return r;
}
inline void backend_live_load(NativeMissionCommandGateway& g) {
  assert(g.load("RUN_H4F_0001","SESSION_LIVE_OPERATOR",
      {{0,0,"P0001","RUN:H4F:P0001"},{100,1,"P0002","RUN:H4F:P0002"}},
      MissionMode::automatic,true,true,true)==ClientCommandStatus::accepted);
}
