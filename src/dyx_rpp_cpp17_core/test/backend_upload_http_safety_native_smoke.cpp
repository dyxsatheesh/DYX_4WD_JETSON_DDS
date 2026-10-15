#include "backend_upload_http_fixture.hpp"
int main(){
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  BackendUploadHttpConfig cfg;
  cfg.max_request_bytes=1024;
  cfg.max_marking_points=2;
  NativeBackendUploadHttpWire wire(backend,cfg);
  auto q=draft_http("POST",kBackendMissionUploadPath,std::string(1025,'A'));
  assert(wire.handle(q).status==413);
  assert(wire.handle(q).http11_bytes().find("413 Content Too Large")!=std::string::npos);
  q.http.body=draft_upload_json(1,"[[0,0],[1,0],[2,0]]");
  assert(wire.handle(q).status==400);
  q.http.body=draft_upload_json(1,"[[0,0],[2,0]]");
  const auto upload=wire.handle(q); assert(upload.status==200);
  auto preview=backend.handle([](){
    BackendMissionUploadRequest r;r.path=kBackendMissionDraftPath;
    r.method="GET";r.mission_run_id="RUN_H4P_0001";
    r.principal={true,true,false,"SESSION_H4P_OPERATOR"};return r;
  }());
  q=draft_http("POST",kBackendMissionLoadPath,draft_load_json(2,1,preview.signature),11);
  q.trusted_px4_disarmed=false;
  assert(wire.handle(q).status==412 && !gateway.has_active_mission());
  q=draft_http("POST",kBackendMissionLoadPath,
      draft_load_json(3,1,std::string(64,'0')),12);
  assert(wire.handle(q).status==409 && !gateway.has_active_mission());
  q=draft_http("POST",kBackendMissionLoadPath,
      draft_load_json(4,1,preview.signature)+"{}",13);
  assert(wire.handle(q).status==400);
  q=draft_http("POST",kBackendMissionLoadPath,
      draft_load_json(5,1,preview.signature),14);
  assert(wire.handle(q).status==200 && gateway.mission().stage()==MissionStage::ready);
  assert(!gateway.emits_hardware_commands());
  // A geometry request that failed validation doesn't mutate an active draft.
  NativeMissionCommandGateway g2;
  NativeBackendMissionUpload backend2(g2);
  NativeBackendUploadHttpWire wire2(backend2);
  auto invalid=draft_upload_json(1,"[[0,0],[2,0]]");
  const auto at=invalid.find("\"extension_distance_m\":4.0");
  assert(at!=std::string::npos);
  invalid.replace(at, std::string("\"extension_distance_m\":4.0").size(),
                  "\"extension_distance_m\":21.0");
  const auto rejected=wire2.handle(draft_http("POST",kBackendMissionUploadPath,invalid));
  assert(rejected.status==422 && !backend2.has_draft());
  assert(rejected.http11_bytes().find("422 Unprocessable Content")!=std::string::npos);
}
