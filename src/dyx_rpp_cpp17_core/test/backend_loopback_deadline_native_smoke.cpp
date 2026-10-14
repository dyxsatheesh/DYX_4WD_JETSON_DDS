#include "backend_loopback_deadline_fixture.hpp"
int main() {
  LoopbackFixture x;
  NativeLoopbackReadOnlyHttpServer listener(x.wire,
      {true,true,false,"SESSION_LOCAL_DEBUG"},{4096,350,500,30});
  const auto port=listener.start();
  const auto start=std::chrono::steady_clock::now();
  const auto client=h4k_connect(port);
  h4k_send(client,"G");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  h4k_send(client,"E");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  h4k_send(client,"T");
  // Continuous activity is not allowed to extend the initial deadline.
  const auto response=h4k_receive(client);
  const auto elapsed=std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now()-start).count();
  assert(elapsed>=240 && elapsed<1600);
  assert(response.find("HTTP/1.1 408 Request Timeout\r\n")==0);
  assert(response.find("HEADER_TOTAL_DEADLINE_EXCEEDED")!=std::string::npos);
  offline_net::close_socket(client);
  assert(loopback_get(port,kBackendMissionStatusPath).find("HTTP/1.1 200 OK") == 0);
  listener.stop();
}
