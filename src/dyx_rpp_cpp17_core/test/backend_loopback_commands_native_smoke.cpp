#include "backend_loopback_fixture.hpp"
int main() {
  LoopbackFixture x;
  const auto port=x.listener.start();
  const auto body=std::string("{\"mission_run_id\":\"RUN_DANGEROUS\",\"sequence\":1}");
  for(const auto* method: {"POST","PUT","DELETE","PATCH"}) {
    const auto req=std::string(method)+" "+kBackendMissionStartPath+" HTTP/1.1\r\nHost: localhost\r\n\r\n";
    const auto r=loopback_request(port,req);
    assert(r.find("HTTP/1.1 405 Method Not Allowed\r\n")==0);
    assert(r.find("READ_ONLY_METHOD_NOT_ALLOWED")!=std::string::npos);
  }
  const auto with_body="POST "+std::string(kBackendMissionStopPath)+" HTTP/1.1\r\nHost: localhost\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
  auto r=loopback_request(port,with_body);
  assert(r.find("HTTP/1.1 400 Bad Request\r\n")==0 ||
         r.find("HTTP/1.1 405 Method Not Allowed\r\n")==0);
  assert(x.gateway.mission().stage()==MissionStage::unloaded);
  x.listener.stop();
}
