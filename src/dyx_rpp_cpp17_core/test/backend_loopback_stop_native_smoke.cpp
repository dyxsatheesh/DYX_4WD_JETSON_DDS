#include "backend_loopback_deadline_fixture.hpp"
int main() {
  LoopbackFixture x;
  bool threw=false;
  try { NativeLoopbackReadOnlyHttpServer bad(x.wire,{}, {64,500,500,30}); }
  catch(const std::invalid_argument&) {threw=true;}
  assert(threw);
  NativeLoopbackReadOnlyHttpServer listener(x.wire,
      {true,true,false,"SESSION_LOCAL_DEBUG"},{4096,2500,600,30});
  auto port=listener.start();
  auto c=h4k_connect(port);
  h4k_send(c,"GET /api/v1/mission/status HTTP/1.1\r\n");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const auto t=std::chrono::steady_clock::now();
  listener.stop(); // must interrupt a slow header reader, not wait 2.5 sec
  const auto took=std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now()-t).count();
  assert(took<1000);
  offline_net::close_socket(c);
  port=listener.start();
  assert(loopback_get(port,kBackendLiveStatusPath).find("HTTP/1.1 200 OK")==0);
  listener.stop();
}
