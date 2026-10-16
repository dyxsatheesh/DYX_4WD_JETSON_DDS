#include "dyx_h6a/launch_gate.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
namespace fs=std::filesystem;
void require(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
void put(const fs::path&p,const std::string&s){fs::create_directories(p.parent_path());std::ofstream(p,std::ios::binary)<<s;}
int main(){
  const fs::path root=fs::temp_directory_path()/("dyx_h6a_test_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto launch=root/"src/dyx_native_observe_bringup/launch/observe.launch.xml";
  const auto obs=root/"src/dyx_jetson_readonly_dds/src/readonly_dds_node.cpp";
  const auto sh=root/"src/dyx_rpp_cpp17_shadow/src/shadow_node.cpp";
  const std::string xml="<launch>\n<node pkg=\"dyx_jetson_readonly_dds\" exec=\"dyx_jetson_readonly_dds_node\" name=\"dyx_jetson_readonly_px4\" output=\"screen\" />\n<node pkg=\"dyx_rpp_cpp17_shadow\" exec=\"dyx_rpp_cpp17_shadow_node\" name=\"dyx_rpp_cpp17_shadow\" output=\"screen\">\n<param name=\"shadow_enabled\" value=\"false\" />\n</node>\n</launch>\n";
  const std::string observer="read only: create_subscription<foo>(\"/fmu/out/status\");\n";
  const std::string shadow="declare_parameter<bool>(\"shadow_enabled\", false);\ncreate_publisher<foo>(\"/dyx/rpp_cpp17_shadow/status\");\n";
  try{
    put(launch,xml);put(obs,observer);put(sh,shadow);
    require(dyx::h6a::inspect(root).pass,"known safe fixture rejected");
    put(launch,xml+"<node pkg=\"mavros\" />\n");
    require(!dyx::h6a::inspect(root).pass,"third node not blocked");
    put(launch,xml);put(sh,shadow+"create_publisher<foo>(\"/fmu/in/trajectory_setpoint\");\n");
    require(!dyx::h6a::inspect(root).pass,"actuation topic not blocked");
    put(sh,shadow);put(obs,observer+"create_publisher<foo>(\"/dyx/sneaky\");\n");
    require(!dyx::h6a::inspect(root).pass,"observer publisher not blocked");
    put(obs,observer);put(launch,xml+"<include file=\"another.xml\" />\n");
    require(!dyx::h6a::inspect(root).pass,"included launch not blocked");
    put(launch,xml);put(sh,shadow+"create_client<foo>(\"/arm\");\n");
    require(!dyx::h6a::inspect(root).pass,"service client not blocked");
    put(sh,shadow);put(launch,xml);require(dyx::h6a::inspect(root).pass,"valid restored fixture rejected");
    fs::remove_all(root);std::cout<<"H6A mutation guard suite passed\n";return 0;
  }catch(const std::exception&e){fs::remove_all(root);std::cerr<<e.what()<<'\n';return 1;}
}
