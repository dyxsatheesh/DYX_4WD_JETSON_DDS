// DEVELOPMENT ONLY: no PX4/Jetson, no synthetic FIXED GNSS data, NO writes.
#include "dyx_rpp_cpp17_core/loopback_readonly_http_server.hpp"
#include <exception>
#include <iostream>
#include <string>
int main(int argc,char**argv) {
  if(argc!=2 || std::string(argv[1])!="--offline-readonly") {
    std::cerr<<"Explicit --offline-readonly required. Do not deploy as a rover server.\n";
    return 2;
  }
  try {
    dyx::rpp::NativeMissionCommandGateway gateway;
    dyx::rpp::NativeBackendMissionApi mission(gateway);
    dyx::rpp::NativeBackendLiveStatus live(gateway);
    dyx::rpp::NativeBackendHttpWire wire(mission,live);
    dyx::rpp::NativeLoopbackReadOnlyHttpServer server(wire,
       {true,true,false,"SESSION_DEMO_READONLY"});
    const auto port=server.start(18765);
    std::cout<<"OFFLINE ONLY: http://127.0.0.1:"<<port<<"/api/v1/mission/status\n"
             <<"No ROS2, no DDS, no mission commands, no actuator control.\n"
             <<"Press Enter to stop.\n"<<std::flush;
    std::string line;std::getline(std::cin,line);
    server.stop();return 0;
  } catch(const std::exception& e) {
    std::cerr<<"OFFLINE server failed: "<<e.what()<<"\n";return 1;
  }
}
