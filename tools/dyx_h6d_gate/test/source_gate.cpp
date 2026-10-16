#include <fstream>
#include <initializer_list>
#include <sstream>
#include <string>
#include <stdexcept>
#include <iostream>
static std::string load(const char *p) {
  std::ifstream f(p,std::ios::binary);
  if(!f)throw std::runtime_error("cannot read H6D node source");
  std::ostringstream s;s<<f.rdbuf();return s.str();
}
static bool safe(const std::string &s) {
  if(s.find("declare_parameter<bool>(\"observe_enabled\", false)")==std::string::npos)
    return false;
  for(const char *required : {"NativeDdsSafetyTelemetry", "NativeSafetyArbiter",
      "observe_battery(f)","observe_gps(f)","observe_vehicle(f)",
      "observe_estimator(f)","corrections_verified=false", "obstacles_verified=false",
      "PHYSICAL_COMMAND_PERMITTED=false"})
    if(s.find(required)==std::string::npos)return false;
  for(const char *bad : {"create_publisher<px4_msgs", "create_publisher<rpp_interfaces",
      "create_publisher<geometry_msgs", "create_service<", "create_client<",
      "\"/fmu/in/", "\"/rpp/command", "\"/spray/", "\"/servo/",
      "observe_corrections(", "observe_obstacle("})
    if(s.find(bad)!=std::string::npos)return false;
  const std::string token="create_publisher<";
  auto first=s.find(token);
  if(first==std::string::npos || s.find(token,first+token.size())!=std::string::npos)
    return false;
  return s.find("create_publisher<std_msgs::msg::String>")==first;
}
int main(int n,char **a) {
  if(n!=2)return 64;
  try {
    const auto source=load(a[1]);
    if(!safe(source))throw std::runtime_error("source failed observation boundary");
    if(safe(source+"\ncreate_publisher<px4_msgs::msg::VehicleCommand>()"))
      throw std::runtime_error("mutation motor publisher not rejected");
    if(safe(source+"\nobserve_corrections(fake)"))
      throw std::runtime_error("mutation RTCM evidence not rejected");
    std::cout<<"H6D SOURCE READ-ONLY GATE PASSED; NOT ROS2/HARDWARE PROOF\n";
    return 0;
  } catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
