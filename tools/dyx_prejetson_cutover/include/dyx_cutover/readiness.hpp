#pragma once
// Static evidence inventory; NEVER vehicle authority or deployment approval.
// Explicitly refuses to infer full feature parity from a filename alone.
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace dyx::cutover {
namespace fs = std::filesystem;

enum class Disposition { blocked_legacy, missing_native, review_required };
inline const char* label(Disposition d) {
  switch (d) {
    case Disposition::blocked_legacy: return "BLOCKED_LEGACY_RUNTIME";
    case Disposition::missing_native: return "MISSING_NATIVE_RUNTIME";
    case Disposition::review_required: return "NATIVE_EXISTS_PARITY_UNVERIFIED";
  }
  return "INVALID";
}
struct Rule {
  std::string id;
  fs::path old_file;
  std::string old_marker;
  fs::path native_file;
};
struct Finding {
  std::string id;
  Disposition disposition{Disposition::missing_native};
  std::string evidence;
};
inline std::string read_file(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}
inline const std::vector<Rule>& rules() {
  static const std::vector<Rule> r = {
    {"backend", "src/rover_backend/setup.py", "rover_backend.main:main", "src/dyx_production_backend/CMakeLists.txt"},
    {"rpp", "src/rpp_controller/setup.py", "rpp_controller.rpp_controller_node:main", "src/dyx_production_rpp/CMakeLists.txt"},
    {"mission", "src/mission_manager/setup.py", "mission_manager", "src/dyx_production_mission/CMakeLists.txt"},
    {"trajectory", "src/trajectory_generator/setup.py", "trajectory_generator", "src/dyx_production_trajectory/CMakeLists.txt"},
    {"ntrip", "src/rtk_correction_bridge/setup.py", "rtk_correction_bridge", "src/dyx_production_ntrip/CMakeLists.txt"},
    {"spray", "src/spray_controller/setup.py", "spray_controller", "src/dyx_production_spray/CMakeLists.txt"},
    {"obstacles", "src/obstacle_monitor/setup.py", "obstacle_monitor", "src/dyx_production_obstacles/CMakeLists.txt"},
    {"localization", "src/localization/setup.py", "localization", "src/dyx_production_localization/CMakeLists.txt"},
    {"motion_transport", "src/jetson_4wd_control/setup.py", "jetson_4wd_control", "src/dyx_production_motion/CMakeLists.txt"},
    {"bringup", "src/rover_bringup/launch/rover.launch.py", "ros2 launch mavros", "src/dyx_production_bringup/launch/rover.launch.xml"},
    {"systemd", "systemd/dyx-rover.service", "scripts/rover_launch.sh", "systemd/dyx-production-native.service"}
  };
  return r;
}
inline std::vector<Finding> scan(const fs::path& root) {
  if (!fs::exists(root / "src") || !fs::exists(root / "tools"))
    throw std::runtime_error("not a DYX source repository: src/ or tools/ is missing");
  std::vector<Finding> result;
  for (const auto& r : rules()) {
    const bool legacy = fs::exists(root/r.old_file) &&
      (r.old_marker.empty() || read_file(root/r.old_file).find(r.old_marker) != std::string::npos);
    const bool native = fs::exists(root/r.native_file);
    const auto d = legacy ? Disposition::blocked_legacy :
      native ? Disposition::review_required : Disposition::missing_native;
    result.push_back({r.id, d, legacy ? r.old_file.generic_string() :
      native ? r.native_file.generic_string() : "native replacement absent"});
  }
  return result;
}
inline std::string json_string(const std::string& s) {
  std::string t="\"";
  const char* hex="0123456789abcdef";
  for (unsigned char c : s) {
    if (c=='\"'||c=='\\') {t+='\\';t+=char(c);}
    else if (c=='\n')t+="\\n";
    else if (c=='\r')t+="\\r";
    else if (c=='\t')t+="\\t";
    else if (c<32) {t+="\\u00";t+=hex[c>>4];t+=hex[c&15];}
    else t+=char(c);
  }
  return t+"\"";
}
inline std::string as_json(const std::vector<Finding>& findings) {
  std::ostringstream o;
  o<<"{\n  \"production_ready\": false,\n  \"limitations\": "
    <<json_string("Static audit only; ROS 2 build, DDS and hardware acceptance are not attested")
    <<",\n  \"findings\": [\n";
  for (size_t i=0;i<findings.size();++i) {
    const auto& f=findings[i];
    o<<"    {\"area\": "<<json_string(f.id)<<", \"status\": "<<json_string(label(f.disposition))
      <<", \"evidence\": "<<json_string(f.evidence)<<"}";
    if(i+1<findings.size())o<<",";
    o<<"\n";
  }
  o<<"  ],\n  \"hardware_acceptance\": \"PENDING\"\n}\n";
  return o.str();
}
} // namespace dyx::cutover
