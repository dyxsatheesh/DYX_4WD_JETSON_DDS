#pragma once
// Offline source/launch boundary check; NOT a ROS2 or hardware safety proof.
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace dyx::h6a {
namespace fs = std::filesystem;
struct Result { bool pass{}; std::vector<std::string> errors; };
inline std::string read(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  if (!in) return {};
  std::ostringstream s; s << in.rdbuf(); return s.str();
}
inline size_t occurrences(const std::string& haystack, const std::string& needle) {
  size_t n=0, from=0;
  while ((from=haystack.find(needle,from))!=std::string::npos) { ++n; from+=needle.size(); }
  return n;
}
inline Result inspect(const fs::path& root) {
  Result r{true,{}};
  const auto fail=[&r](const std::string& error){ r.pass=false; r.errors.push_back(error); };
  const std::string xml=read(root/"src/dyx_native_observe_bringup/launch/observe.launch.xml");
  const std::string observer=read(root/"src/dyx_jetson_readonly_dds/src/readonly_dds_node.cpp");
  const std::string shadow=read(root/"src/dyx_rpp_cpp17_shadow/src/shadow_node.cpp");
  if (xml.empty()||observer.empty()||shadow.empty()) { fail("required launch or source missing"); return r; }
  // Entire launch must contain only the two exact known read-only binaries;
  // any additional node, include, command or executable fails closed.
  const std::string first="<node pkg=\"dyx_jetson_readonly_dds\" exec=\"dyx_jetson_readonly_dds_node\" name=\"dyx_jetson_readonly_px4\" output=\"screen\" />";
  const std::string second="<node pkg=\"dyx_rpp_cpp17_shadow\" exec=\"dyx_rpp_cpp17_shadow_node\" name=\"dyx_rpp_cpp17_shadow\" output=\"screen\">";
  if (occurrences(xml,"<node ")!=2 || occurrences(xml,first)!=1 || occurrences(xml,second)!=1)
    fail("launch must contain precisely the two approved observation nodes");
  if (occurrences(xml,"<launch>")!=1 || occurrences(xml,"</launch>")!=1)
    fail("launch structure invalid");
  if (occurrences(xml,"<param name=\"shadow_enabled\" value=\"false\" />")!=1)
    fail("shadow preview must be disabled by default");
  for (const std::string forbidden : {"<include", "<executable", "<group", "<let", "<arg", "<env", "<set_env", "<remap", "<param_file", "/fmu/in/", "mavros", "dyx_px4_dds_bridge", "enable_motion", "true\""}) {
    if (xml.find(forbidden)!=std::string::npos) fail("forbidden launch construct: "+forbidden);
  }
  // No outgoing PX4 command publisher in either source. The shadow's sole
  // publisher is its diagnostic string, with no other outgoing topic.
  for (const std::string* source : {&observer,&shadow}) {
    for (const std::string forbidden : {"\"/fmu/in/", "\"/rpp/command", "create_client<", "create_service<", "async_send_request", "publish_setpoint("}) {
      if (source->find(forbidden)!=std::string::npos) fail("unexpected output/control in observed C++ source: "+forbidden);
    }
  }
  if (occurrences(observer,"create_publisher<")!=0)
    fail("PX4 observer must not publish any ROS topic");
  if (occurrences(shadow,"create_publisher<")!=1 || shadow.find("\"/dyx/rpp_cpp17_shadow/status\"")==std::string::npos)
    fail("RPP shadow must publish only its diagnostic status");
  if (shadow.find("declare_parameter<bool>(\"shadow_enabled\", false)")==std::string::npos)
    fail("shadow-enabled default must remain false");
  return r;
}
} // namespace dyx::h6a
