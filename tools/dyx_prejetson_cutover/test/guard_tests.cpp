#include "dyx_cutover/readiness.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs=std::filesystem;
static void put(const fs::path& p,const std::string& s) {
  fs::create_directories(p.parent_path());
  std::ofstream f(p); f<<s; assert(bool(f));
}
int main() {
  const auto token=std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root=fs::temp_directory_path()/("dyx_cutover_test_"+std::to_string(token));
  fs::create_directories(root/"tools");fs::create_directories(root/"src");
  // A genuine MAVROS launch must be blocked even if an unrelated native
  // observer node happens to exist.
  put(root/"src/rover_bringup/launch/rover.launch.py", "ros2 launch mavros");
  put(root/"src/dyx_jetson_readonly_dds/CMakeLists.txt", "observer only");
  auto v=dyx::cutover::scan(root);
  assert(v.size()==dyx::cutover::rules().size());
  assert(v[9].id=="bringup" && v[9].disposition==dyx::cutover::Disposition::blocked_legacy);
  assert(v[0].disposition==dyx::cutover::Disposition::missing_native);
  assert(dyx::cutover::as_json(v).find("\"production_ready\": false")!=std::string::npos);
  fs::remove(root/"src/rover_bringup/launch/rover.launch.py");
  put(root/"src/dyx_production_bringup/launch/rover.launch.xml", "<launch/>");
  v=dyx::cutover::scan(root);
  assert(v[9].disposition==dyx::cutover::Disposition::review_required);
  assert(dyx::cutover::json_string("x\n\"\\")=="\"x\\n\\\"\\\\\"");
  fs::remove_all(root);
  bool rejected=false;
  try { (void)dyx::cutover::scan(root); } catch(const std::runtime_error&){rejected=true;}
  assert(rejected);
}
