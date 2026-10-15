#include "safety.hpp"
#include "systemd_notify.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
namespace {
volatile std::sig_atomic_t stopping = 0;
extern "C" void stop_signal(int) { stopping = 1; }
}
int main(int argc, char** argv) {
  const dyx::preflight::InhibitState state;
  if(argc==2 && std::strcmp(argv[1],"--check")==0) {
    std::cout<<state.json()<<'\n'; return 0;
  }
  if(argc!=2 || std::strcmp(argv[1],"--run")!=0) {
    std::cerr<<"Usage: dyx_jetson_preflight --check | --run\n"; return 64;
  }
  std::signal(SIGTERM,stop_signal); std::signal(SIGINT,stop_signal);
  unsigned long long us=0;
  if(dyx::preflight::watchdog_requested() && !dyx::preflight::watchdog_usecs(us)) {
    std::cerr << "Invalid systemd watchdog environment; fail closed\n";
    return 70;
  }
  if(std::getenv("NOTIFY_SOCKET") &&
     !dyx::preflight::notify_systemd("READY=1\nSTATUS=OBSERVE_ONLY_OUTPUTS_INHIBITED")) {
    std::cerr << "Cannot notify systemd readiness\n";return 70;
  }
  if(us && !std::getenv("NOTIFY_SOCKET")) {
    std::cerr << "Watchdog requested without notification endpoint\n";return 70;
  }
  std::cout<<state.json()<<std::endl;
  const auto wait=std::chrono::microseconds(us ? us/4 : 100000ULL);
  while(!stopping) {
    std::this_thread::sleep_for(wait);
    if(us && !dyx::preflight::notify_systemd("WATCHDOG=1")) {
      std::cerr << "Watchdog notification failure; fail closed\n";return 70;
    }
  }
  if(std::getenv("NOTIFY_SOCKET"))
    (void)dyx::preflight::notify_systemd("STOPPING=1");
  return 0;
}
