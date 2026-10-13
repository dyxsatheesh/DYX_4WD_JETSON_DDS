#include "backend_http_fixture.hpp"
#include <vector>
int main() {
  NativeMissionCommandGateway g;
  http_load(g);
  NativeBackendMissionApi commands(g);
  NativeBackendLiveStatus live(g);
  NativeBackendHttpWire wire(commands,live);
  auto r=http_read(kBackendLiveStatusPath,1.0);
  r.principal.authenticated=false;
  auto noauth=wire.handle(r);
  assert(noauth.status==401);
  assert(noauth.body=="{\"ok\":false,\"code\":\"AUTHENTICATION_REQUIRED\"}");
  assert(noauth.body.find("P0001")==std::string::npos);
  assert(wire.make_live_sse_frame(r,1).empty());
  r.principal.authenticated=true; r.principal.can_read_mission=false;
  assert(wire.handle(r).status==403);
  r=http_read(kBackendLiveStatusPath,1.0); r.method="POST";
  assert(wire.handle(r).status==405);
  r=http_read("/api/v1/telemetry/other",1.0);
  assert(wire.handle(r).status==404);
  r=http_read(kBackendMissionStatusPath,1.0);r.body="{}";
  assert(wire.handle(r).status==400);
  auto post=http_post(kBackendMissionStartPath,1,1.0);
  post.principal.authenticated=false;
  assert(wire.handle(post).status==401);
  post=http_post(kBackendMissionStartPath,1,1.0);
  post.principal.owns_control_lease=false;
  assert(wire.handle(post).status==403);
  post=http_post(kBackendMissionStartPath,1,1.0);
  const std::vector<std::string> invalid={
    "{\"mission_run_id\":\"RUN_HTTP_0001\",\"sequence\":1,\"authenticated\":true}",
    "{\"mission_run_id\":\"RUN_HTTP_0001\",\"sequence\":1,\"sequence\":2}",
    "{\"mission_run_id\":\"RUN_HTTP_0001\",\"sequence\":-1}",
    "{\"mission_run_id\":\"RUN_HTTP_0001\",\"sequence\":1.0}",
    "{\"mission_run_id\":\"RUN_HTTP_0001\",\"sequence\":1e1}",
    "{\"mission_run_id\":\"RUN_HTTP_0001\",\"sequence\":01}",
    "{\"mission_run_id\":\"RUN_HTTP_0001\",\"sequence\":0}",
    "{\"mission_run_id\":\"RUN_HTTP_0001\",\"sequence\":18446744073709551616}",
    "{\"mission_run_id\":\"RUN_HTTP_0001\",\"sequence\":\"1\"}",
    "{\"mission_run_id\":\"RUN_HTTP_0001\",\"sequence\":1} garbage",
    "{\"mission_run_id\":\"RUN\\u005fHTTP_0001\",\"sequence\":1}",
    "{\"mission_run_id\":\"RUN_HTTP_0001\"}",
    "[]","null","", "{\"sequence\":1,\"mission_run_id\":\"RUN_HTTP_0001\",}"
  };
  for (const auto& body: invalid) {
    post.body=body;
    const auto result=wire.handle(post);
    assert(result.status==400);
    assert(result.body.find("INVALID_COMMAND_JSON")!=std::string::npos);
    assert(g.mission().stage()==MissionStage::ready);
  }
  post.body=std::string(513,'A');
  assert(wire.handle(post).status==413);
  post=http_post(kBackendMissionStartPath,1,1.0);
  post.body="{\"sequence\":1,\"mission_run_id\":\"RUN_HTTP_0001\"}";
  // Both key orders are deliberately accepted, but not authorization via JSON.
  assert(wire.handle(post).status==412);
  assert(g.mission().stage()==MissionStage::ready);
}
