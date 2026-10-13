#pragma once
#include "mission_command_gateway_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_mission_api.hpp"
#include <cassert>
#include <limits>
using namespace dyx::rpp;
inline BackendMissionApiRequest api_req(const char* path, unsigned seq, double t,
                                        const char* method="POST") {
  BackendMissionApiRequest r;
  r.method=method;
  r.path=path;
  r.mission_run_id=gateway_run;
  r.sequence=seq;
  r.host_received_sec=t;
  r.principal={true,true,true,gateway_session};
  return r;
}
inline BackendMissionApiRequest api_status_req() {
  auto r=api_req(kBackendMissionStatusPath,0,0,"GET");
  r.principal.owns_control_lease=false;
  return r;
}
