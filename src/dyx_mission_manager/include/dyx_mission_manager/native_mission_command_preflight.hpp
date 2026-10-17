#pragma once
// Batch 7B: authenticated mission-command envelope and hard execution lockout.
// No ROS gateway dispatch, arming, or actuator emission is permitted.
#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>
#include <limits>
#include "dyx_mission_manager/native_execution_preflight.hpp"

namespace dyx::http_backend {
class NativeMissionCommandPreflight final {
 public:
  struct Validated { bool ok{false}; std::string run_id; std::uint64_t sequence{0}; };
  static Validated validate(const nlohmann::json& j) {
    Validated out{};
    if (!j.is_object() || j.size()!=2 || !j.contains("mission_run_id") ||
        !j.contains("sequence") || !j.at("mission_run_id").is_string() ||
        !j.at("sequence").is_number_integer()) return out;
    const auto& id = j.at("mission_run_id").get_ref<const std::string&>();
    if (id.empty() || id.size()>128) return out;
    for (const char c : id)
      if (!(c>='a'&&c<='z') && !(c>='A'&&c<='Z') &&
          !(c>='0'&&c<='9') && c!='_' && c!='-') return out;
    if (!j.at("sequence").is_number_unsigned() &&
        j.at("sequence").get<std::int64_t>()<=0) return out;
    const auto sequence = j.at("sequence").get<std::uint64_t>();
    if (sequence == 0) return out;
    out.ok=true; out.run_id=id; out.sequence=sequence;
    return out;
  }
  static nlohmann::json deny(const nlohmann::json& snapshot) {
    const auto preflight=NativeExecutionPreflight::evaluate(snapshot);
    // Even a fully valid diagnostic preflight does not yet grant motion.
    return {{"state","COMMAND_BLOCKED"},{"detail","EXECUTION_GATEWAY_NOT_COMMISSIONED"},
            {"safety_preflight",preflight},{"decision_only",true},
            {"command_queued",false},{"mission_commands_enabled",false},
            {"motor_commands_enabled",false},{"spray_commands_enabled",false},
            {"physical_command_emitted",false}};
  }
};
} // namespace dyx::http_backend
