#include "backend_loopback_fixture.hpp"
int main() {
  LoopbackFixture x;
  NativeLoopbackReadOnlyHttpServer denied(x.wire);
  auto port=denied.start();
  auto r=loopback_get(port,kBackendLiveStatusPath);
  assert(r.find("HTTP/1.1 401 Unauthorized\r\n")==0);
  denied.stop();
  port=denied.start();
  r=loopback_get(port,kBackendMissionStatusPath);
  assert(r.find("HTTP/1.1 401 Unauthorized\r\n")==0);
  denied.stop();
  port=x.listener.start();
  r=loopback_get(port,kBackendMissionStatusPath);
  assert(r.find("HTTP/1.1 200 OK\r\n")==0);
  x.listener.stop();
  x.listener.stop(); // idempotent
}
