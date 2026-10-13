#pragma once
#include "dyx_rpp_cpp17_core/mission_command_gateway.hpp"
#include <cassert>
#include <cstdint>
using namespace dyx::rpp;

inline constexpr const char *gateway_run="run_2H4C_0001";
inline constexpr const char *gateway_session="session_2H4C_01";
inline void gateway_load(NativeMissionCommandGateway &g,
                         MissionMode mode=MissionMode::automatic,
                         const char *run=gateway_run,
                         const char *session=gateway_session) {
  assert(g.load(run, session, {{0,0,"P0001","RUN:H4C:P0001"},
                                {100,1,"P0002","RUN:H4C:P0002"}},
                mode,true,true,true)==ClientCommandStatus::accepted);
}
inline std::uint64_t gateway_stamp(double t){return static_cast<std::uint64_t>((t+10.0)*1000000.0);}
inline void gateway_feed(NativeMissionCommandGateway &g,double t,int fix=6,
                         bool obstacle_clear=true){
  const auto stamp=gateway_stamp(t);
  assert(g.observe_vehicle({stamp,t,2,14,3,false,0})==DdsTelemetryDecision::accepted);
  assert(g.observe_battery({stamp,t,24.9,true,false,8,0})==DdsTelemetryDecision::accepted);
  assert(g.observe_gps({stamp,t,0x1234,12.5,77.5,fix,false,0})==DdsTelemetryDecision::accepted);
  assert(g.observe_estimator({stamp,t,true,true,true,false,0.2,0.1,0.0})==DdsTelemetryDecision::accepted);
  assert(g.observe_corrections({t,0.2,true})==DdsTelemetryDecision::accepted);
  assert(g.observe_obstacle({t,obstacle_clear})==DdsTelemetryDecision::accepted);
}
inline DdsMissionRuntimeOutput gateway_tick(NativeMissionCommandGateway &g,double t,
                                           bool motion_enabled=true,
                                           bool estop=false){
  MissionSequenceInput m;
  m.monotonic_time_sec=t;
  m.emergency_stop=estop;
  m.measured_forward_speed_mps=0.4;
  return g.tick(t,true,motion_enabled,estop,m);
}
inline ClientCommandRequest gateway_request(ClientMissionAction action,
                                             std::uint64_t sequence,double t){
  ClientCommandRequest request;
  request.action=action;
  request.mission_run_id=gateway_run;
  request.operator_session_id=gateway_session;
  request.session_sequence=sequence;
  request.received_at_host_sec=t;
  request.authenticated_operator=true;
  request.authorized_control_lease=true;
  return request;
}
