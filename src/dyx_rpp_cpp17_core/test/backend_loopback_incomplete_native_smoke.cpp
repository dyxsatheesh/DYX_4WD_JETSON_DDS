#include "backend_loopback_deadline_fixture.hpp"
int main() {
  LoopbackFixture x;
  NativeLoopbackReadOnlyHttpServer listener(x.wire,
      {true,true,false,"SESSION_LOCAL_DEBUG"},{4096,500,500,30});
  const auto port=listener.start();
  auto c=h4k_connect(port);
  h4k_send(c,"GET /api/v1/mission/status HTTP/1.1\r\nHost: localhost\r\n");
#ifdef _WIN32
  assert(::shutdown(c,SD_SEND)==0);
#else
  assert(::shutdown(c,SHUT_WR)==0);
#endif
  const auto reply=h4k_receive(c);
  assert(reply.find("HTTP/1.1 400 Bad Request\r\n")==0);
  assert(reply.find("INCOMPLETE_REQUEST_HEADERS")!=std::string::npos);
  offline_net::close_socket(c);
  // The incomplete client must not poison the next request.
  assert(loopback_get(port,kBackendLiveStatusPath).find("HTTP/1.1 200 OK") == 0);
  listener.stop();
}
