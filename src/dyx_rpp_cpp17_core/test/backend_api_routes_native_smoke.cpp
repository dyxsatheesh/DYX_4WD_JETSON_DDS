#include "backend_api_fixture.hpp"
int main() {
  NativeMissionCommandGateway g;
  NativeBackendMissionApi api(g);
  assert(api.handle(api_status_req()).http_status==200);
  assert(api.handle(api_status_req()).stage==MissionStage::unloaded);
  auto request=api_req("/api/v1/mission/start/",1,0);
  assert(api.handle(request).http_status==404);
  request.path="/api/v1/mission/start?debug=1";
  assert(api.handle(request).http_status==404);
  request.path=kBackendMissionStartPath;
  request.method="GET";
  assert(api.handle(request).http_status==405);
  request.method="POST";
  assert(api.handle(request).http_status==409); // no active mission
  assert(!g.has_active_mission());
  gateway_load(g);
  const auto before=g.mission().stage();
  auto read=api_status_req();
  read.mission_run_id="untrusted_other_run";
  read.sequence=1234;
  const auto status=api.handle(read);
  assert(status.http_status==200 && status.stage==before);
  assert(status.total==2 && status.pending==2 && status.active_marking_label=="P0001");
  assert(!status.hardware_command_emitted && status.decision_only);
  assert(g.mission().stage()==before); // GET cannot start a mission
}
