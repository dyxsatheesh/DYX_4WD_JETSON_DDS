#include "backend_lifecycle_fixture.hpp"
#include <array>
int main() {
  NativeOfflineRequestLifecycle lifecycle;
  std::uint64_t id=1;
  double t=1.0;
  for (auto route : {kBackendMissionStatusPath,kBackendLiveStatusPath}) {
    auto r=lifecycle.begin(offline_get(id++,route,t),t);
    assert(r.allowed());
    assert(lifecycle.complete(r.ticket,t,128).code==OfflineLifecycleCode::completed);
    t+=0.1;
  }
  for (auto route : {kBackendMissionStartPath,kBackendMissionPausePath,
                     kBackendMissionResumePath,kBackendMissionNextPath,
                     kBackendMissionSkipPath}) {
    auto r=lifecycle.begin(offline_post(id++,route,t),t);
    assert(r.allowed());
    assert(lifecycle.complete(r.ticket,t,128).code==OfflineLifecycleCode::completed);
    t+=0.1;
  }
  auto bad=offline_post(id++,kBackendMissionStartPath,t);
  bad.method="GET";
  assert(lifecycle.begin(bad,t).code==OfflineLifecycleCode::method_not_allowed);
  bad=offline_post(id++,kBackendMissionStartPath,t);
  bad.content_type="text/plain";
  assert(lifecycle.begin(bad,t).code==OfflineLifecycleCode::invalid_content_type);
  bad=offline_post(id++,kBackendMissionStartPath,t);
  bad.body="{\"mission_run_id\":\"RUN_LIFECYCLE_0001\",\"sequence\":1,\"authenticated\":true}";
  assert(lifecycle.begin(bad,t).code==OfflineLifecycleCode::invalid_command_json);
  bad=offline_post(id++,kBackendMissionStartPath,t);
  bad.body=std::string(513,'x');
  assert(lifecycle.begin(bad,t).code==OfflineLifecycleCode::request_too_large);
  bad=offline_get(id++,"/api/v1/telemetry/live?auth=true",t);
  assert(lifecycle.begin(bad,t).code==OfflineLifecycleCode::unsupported_route);
  bad=offline_get(id++,kBackendMissionStatusPath,t);
  bad.body="{}";
  assert(lifecycle.begin(bad,t).code==OfflineLifecycleCode::invalid_command_json);
  const auto stop=lifecycle.begin(offline_post(id++,kBackendMissionStopPath,t),t);
  assert(stop.code==OfflineLifecycleCode::stop_out_of_band);
}
