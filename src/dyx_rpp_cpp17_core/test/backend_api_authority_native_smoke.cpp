#include "backend_api_fixture.hpp"
int main() {
  NativeMissionCommandGateway g;
  NativeBackendMissionApi api(g);
  gateway_load(g);
  gateway_feed(g,1);gateway_tick(g,1);
  auto status=api_status_req();
  status.principal.authenticated=false;
  assert(api.handle(status).http_status==401);
  status.principal.authenticated=true;
  status.principal.can_read_mission=false;
  assert(api.handle(status).http_status==403);
  auto start=api_req(kBackendMissionStartPath,1,1.1);
  start.principal.authenticated=false;
  assert(api.handle(start).http_status==401);
  start.principal.authenticated=true;
  start.principal.owns_control_lease=false;
  assert(api.handle(start).http_status==403);
  start.principal.owns_control_lease=true;
  start.principal.operator_session_id="short";
  assert(api.handle(start).http_status==400);
  start.principal.operator_session_id=gateway_session;
  start.mission_run_id="invalid space";
  assert(api.handle(start).http_status==400);
  start.mission_run_id=gateway_run;
  start.host_received_sec=std::numeric_limits<double>::quiet_NaN();
  assert(api.handle(start).http_status==400);
  start.host_received_sec=1.1;
  assert(api.handle(start).http_status==200); // Denials did not consume safety tick
  assert(g.mission().stage()==MissionStage::approaching);
}
