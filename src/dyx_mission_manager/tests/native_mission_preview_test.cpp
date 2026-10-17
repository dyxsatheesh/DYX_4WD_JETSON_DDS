#include "dyx_mission_manager/native_mission_preview.hpp"
#include <cassert>
#include <iostream>
using nlohmann::json;
int main() {
  const json base = {
      {"mission_run_id", "native_run_0001"},
      {"mission_mode", "automatic"},
      {"marking_points", json::array({
          {{"x_m", 0.0}, {"y_m", 0.0}},
          {{"x_m", 1.0}, {"y_m", 0.0}},
          {{"x_m", 2.0}, {"y_m", 0.0}}
      })}};
  const auto ok = dyx::http_backend::NativeMissionPreview::validate(base);
  assert(ok.http_status == 200);
  assert(ok.body.at("marking_count") == 3);
  assert(ok.body.at("navigation_point_count").get<int>() > 3);
  assert(ok.body.at("signature").get<std::string>().size() == 64);
  assert(ok.body.at("mission_loaded") == false);
  assert(ok.body.at("motor_commands_enabled") == false);
  assert(ok.body.at("physical_command_emitted") == false);
  auto two = base;
  two["marking_points"].erase(two["marking_points"].begin() + 2);
  assert(dyx::http_backend::NativeMissionPreview::validate(two).http_status == 200);
  auto bad = base;
  bad["trusted_px4_disarmed"] = true;
  assert(dyx::http_backend::NativeMissionPreview::validate(bad).http_status == 422);
  bad = base;
  bad["marking_points"][1]["x_m"] = "1.0";
  assert(dyx::http_backend::NativeMissionPreview::validate(bad).http_status == 422);
  bad = base;
  bad["marking_points"][1] = bad["marking_points"][0];
  assert(dyx::http_backend::NativeMissionPreview::validate(bad).http_status == 422);
  bad = base;
  bad["extension_distance_m"] = 20.1;
  assert(dyx::http_backend::NativeMissionPreview::validate(bad).http_status == 422);
  bad = base;
  bad["mission_run_id"] = "bad";
  assert(dyx::http_backend::NativeMissionPreview::validate(bad).http_status == 422);
  bad = base;
  bad["mission_mode"] = "unsupported";
  assert(dyx::http_backend::NativeMissionPreview::validate(bad).http_status == 422);
  bad = base;
  bad["owns_control_lease"] = true;
  assert(dyx::http_backend::NativeMissionPreview::validate(bad).http_status == 422);
  std::cout << "PASS: C++17 ENU preview, signature, bindings, two-point path, schema and authority-spoof rejection\n";
}
