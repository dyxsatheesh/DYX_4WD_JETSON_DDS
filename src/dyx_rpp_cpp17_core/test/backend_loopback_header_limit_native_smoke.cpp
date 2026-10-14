#include "backend_loopback_deadline_fixture.hpp"
int main() {
  LoopbackFixture x;
  NativeLoopbackReadOnlyHttpServer listener(x.wire,
      {true,true,false,"SESSION_LOCAL_DEBUG"},{512,500,500,30});
  const auto port=listener.start();
  const auto big="GET /api/v1/mission/status HTTP/1.1\r\nHost: localhost\r\nX-Pad: " +
      std::string(1200,'A') + "\r\n\r\n";
  auto reply=loopback_request(port,big);
  assert(reply.find("HTTP/1.1 413 Content Too Large\r\n")==0);
  assert(reply.find("REQUEST_HEADERS_TOO_LARGE")!=std::string::npos);
  // One packet containing headers plus body/pipelined bytes fails closed.
  reply=loopback_request(port,
      "GET /api/v1/mission/status HTTP/1.1\r\nHost: localhost\r\n\r\nBAD");
  assert(reply.find("HTTP/1.1 400 Bad Request\r\n")==0);
  assert(reply.find("REQUEST_BODY_NOT_ALLOWED")!=std::string::npos);
  assert(loopback_get(port,kBackendMissionStatusPath).find("HTTP/1.1 200 OK")==0);
  listener.stop();
}
