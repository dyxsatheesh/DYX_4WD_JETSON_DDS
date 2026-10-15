#include "backend_upload_http_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  const auto good=draft_upload_json();
  auto reject=[&](const std::string& body){
    auto r=wire.handle(draft_http("POST",kBackendMissionUploadPath,body));
    assert(r.status==400);
    assert(!backend.has_draft());
  };
  reject(good+" "+"true");
  reject("{}");
  reject(good.substr(0,good.size()-1));
  reject("{\"mission_run_id\":\"RUN_H4P_0001\",\"mission_run_id\":\"RUN_H4P_0001\"}");
  reject(good.substr(0,good.size()-1)+",\"authenticated\":true}");
  reject(good.substr(0,good.size()-1)+",\"trusted_px4_disarmed\":true}");
  reject(good.substr(0,good.size()-1)+",\"principal\":{}}");
  reject(good.substr(0,good.size()-1)+",\"sequence\":1}");
  reject(draft_upload_json(0));
  reject(draft_upload_json(1,"[[NaN,0],[2,0]]"));
  reject(draft_upload_json(1,"[[0,0],[1e309,0]]"));
  reject(draft_upload_json(1,"[[0,0],[2,0],]"));
  reject(draft_upload_json(1,"[[0,0],[2,0],[3]]"));
  reject(draft_upload_json(1,"[[0,0],[2,0],[3,4,5]]"));
  reject(draft_upload_json(1,"[[0,0],[+2,0]]"));
  reject(draft_upload_json(1,"[[0,0],[02,0]]"));
  reject(draft_upload_json(1,"[[0,0],[.2,0]]"));
  reject(draft_upload_json(1,"[[0,0],[2,0]]").substr(0,25));
  auto q=draft_get();q.http.path+="&admin=true";
  assert(wire.handle(q).status==400);
  q=draft_get();q.http.body="{}";
  assert(wire.handle(q).status==400);
  q=draft_get();q.http.method="POST";
  assert(wire.handle(q).status==405);
  assert(wire.handle(draft_http("POST","/api/v1/mission/nope",good)).status==404);
  assert(wire.handle(draft_http("GET",kBackendMissionUploadPath,good)).status==405);
  q=draft_http("POST",kBackendMissionUploadPath,good);q.http.host_received_sec=-1;
  assert(wire.handle(q).status==400);
  assert(!backend.has_draft());
}
