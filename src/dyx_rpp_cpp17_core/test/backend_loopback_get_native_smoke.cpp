#include "backend_loopback_fixture.hpp"
int main() {
  LoopbackFixture x;
  const auto port=x.listener.start();
  assert(port!=0);
  const auto status=loopback_get(port,kBackendMissionStatusPath);
  assert(status.find("HTTP/1.1 200 OK\r\n")==0);
  assert(status.find("\"stage\":\"unloaded\"")!=std::string::npos);
  assert(status.find("\"hardware_command_emitted\":false")!=std::string::npos);
  assert(status.find("Connection: close\r\n")!=std::string::npos);
  const auto live=loopback_get(port,kBackendLiveStatusPath);
  assert(live.find("HTTP/1.1 200 OK\r\n")==0);
  assert(live.find("\"rtk\":\"UNAVAILABLE\"")!=std::string::npos);
  assert(live.find("\"marking_accuracy_m\":null")!=std::string::npos);
  x.listener.stop();
}
