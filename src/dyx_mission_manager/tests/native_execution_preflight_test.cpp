#include "dyx_mission_manager/native_execution_preflight.hpp"
#include <cassert>
#include <algorithm>
#include <iostream>
using dyx::http_backend::NativeExecutionPreflight;
static bool blocked(const nlohmann::json& j, const std::string& code) {
  for (const auto& value: j.at("blockers"))
    if (value.get<std::string>() == code) return true;
  return false;
}
int main() {
  auto a = NativeExecutionPreflight::evaluate(nlohmann::json::object());
  assert(a["state"] == "BLOCKED" && blocked(a, "RTK_FIX_NOT_VERIFIED"));
  assert(blocked(a, "PHYSICAL_ESTOP_UNVERIFIED"));
  assert(a["ready_for_physical_motion"] == false && a["start_enabled"] == false);
  nlohmann::json spoof = {{"mission_loaded",true},{"physical_estop_clear_verified",true},
     {"rtk_fixed",true},{"px4_control_interface_commissioned",true}};
  assert(NativeExecutionPreflight::evaluate(spoof)["state"] == "BLOCKED");
  nlohmann::json complete = nlohmann::json::object();
  for (auto key: {"mission_loaded","px4_vehicle_fresh","gps_fresh","battery_fresh",
  "rear_reference_calibrated","rtk_fixed","corrections_fresh","obstacle_clear_verified",
  "physical_estop_clear_verified","operator_motion_authorized","battery_reserve_ok",
  "px4_control_interface_commissioned"}) complete[key] = true;
  auto diagnostic = NativeExecutionPreflight::evaluate(complete);
  assert(diagnostic["state"] == "CHECKS_SATISFIED_DIAGNOSTIC_ONLY");
  assert(diagnostic["ready_for_physical_motion"] == false);
  complete["safe_stop_requested"] = true;
  assert(blocked(NativeExecutionPreflight::evaluate(complete),"STOP_REQUESTED"));
  complete["safe_stop_requested"] = false;
  complete["telemetry_fault"] = true;
  assert(blocked(NativeExecutionPreflight::evaluate(complete),"TELEMETRY_FAULT_LATCHED"));
  complete["telemetry_fault"] = false;
  complete["px4_control_interface_commissioned"] = "yes";
  assert(blocked(NativeExecutionPreflight::evaluate(complete),"PX4_CONTROL_INTERFACE_NOT_COMMISSIONED"));
  std::cout << "PASS: Preflight fail-closed on missing/wrong-type evidence, STOP, telemetry faults; no motion authorization\n";
}
