#pragma once
// Decision-only preflight report. NO motion permission or PX4 publisher.
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace dyx::http_backend {
class NativeExecutionPreflight final {
 public:
  static nlohmann::json evaluate(const nlohmann::json& s) {
    std::vector<std::string> blockers;
    if (!s.is_object()) blockers.emplace_back("ROS_SNAPSHOT_INVALID");
    const auto has = [&s](const char* key) {
      const auto it = s.is_object() ? s.find(key) : s.end();
      return it != s.end() && it->is_boolean() && it->get<bool>();
    };
    const auto reject = [&blockers, &has](const char* key, const char* code) {
      if (!has(key)) blockers.emplace_back(code);
    };
    reject("mission_loaded", "MISSION_NOT_LOADED");
    reject("px4_vehicle_fresh", "PX4_VEHICLE_STALE");
    reject("gps_fresh", "GNSS_STALE");
    reject("battery_fresh", "BATTERY_STALE");
    reject("rear_reference_calibrated", "REAR_REFERENCE_NOT_CALIBRATED");
    reject("rtk_fixed", "RTK_FIX_NOT_VERIFIED");
    reject("corrections_fresh", "RTCM_STALE_OR_UNVERIFIED");
    reject("obstacle_clear_verified", "OBSTACLE_CLEARANCE_UNVERIFIED");
    reject("physical_estop_clear_verified", "PHYSICAL_ESTOP_UNVERIFIED");
    reject("operator_motion_authorized", "OPERATOR_MOTION_NOT_AUTHORIZED");
    reject("battery_reserve_ok", "BATTERY_RESERVE_UNVERIFIED");
    reject("px4_control_interface_commissioned", "PX4_CONTROL_INTERFACE_NOT_COMMISSIONED");
    if (has("safe_stop_requested")) blockers.emplace_back("STOP_REQUESTED");
    if (has("telemetry_fault")) blockers.emplace_back("TELEMETRY_FAULT_LATCHED");
    if (has("position_fault")) blockers.emplace_back("POSITION_FAULT_LATCHED");
    if (has("control_timing_fault")) blockers.emplace_back("CONTROL_TIMING_FAULT_LATCHED");
    if (has("mailbox_stop_latched")) blockers.emplace_back("STOP_MAILBOX_LATCHED");
    // Explicit readiness is diagnostics only. Never connect this to actuation.
    return {{"state", blockers.empty() ? "CHECKS_SATISFIED_DIAGNOSTIC_ONLY" : "BLOCKED"},
            {"blockers", blockers}, {"ready_for_physical_motion", false},
            {"start_enabled", false}, {"resume_enabled", false},
            {"px4_setpoints_published", false}, {"spray_commands_enabled", false},
            {"decision_only", true}};
  }
};
}  // namespace dyx::http_backend
