#include "backend_loopback_fixture.hpp"
int main() {
  LoopbackFixture x;
  const auto port=x.listener.start();
  const auto path=std::string(kBackendLiveStatusPath);
  for(const auto& extra: {
      std::string("Host: evil.example\r\n"),
      std::string("Host: localhost\r\nHost: localhost\r\n"),
      std::string("Host: localhost\r\nTransfer-Encoding: chunked\r\n"),
      std::string("Host: localhost\r\nContent-Length: 0\r\n"),
      std::string("Host: localhost\r\n X-Smuggled: yes\r\n")}) {
    auto r=loopback_request(port,"GET "+path+" HTTP/1.1\r\n"+extra+"\r\n");
    assert(r.find("HTTP/1.1 400 Bad Request\r\n")==0);
  }
  auto wrong_path=loopback_get(port,"/api/v1/mission/start");
  assert(wrong_path.find("HTTP/1.1 405 Method Not Allowed\r\n")==0);
  const auto valid=loopback_get(port,kBackendMissionStatusPath);
  assert(valid.find("HTTP/1.1 200 OK\r\n")==0);
  x.listener.stop();
}
