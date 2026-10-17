#pragma once
// Authenticated PREVIEW of LOCAL ENU marking geometry only.
// This is not a mission upload, trusted PX4-disarm attestation, control lease,
// arm/start authorization, navigation setpoint, or spray command.

#include "dyx_rpp_cpp17_core/mission_runtime_install.hpp"
#include "dyx_rpp_cpp17_core/trajectory_builder.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <exception>
#include <string>
#include <utility>
#include <vector>

namespace dyx::http_backend {

struct NativeMissionPreviewResult {
  int http_status{422};
  nlohmann::json body{};
};

class NativeMissionPreview final {
 public:
  static NativeMissionPreviewResult validate(const nlohmann::json& request) {
    constexpr const char* kInvalid = "INVALID_NATIVE_MISSION_PREVIEW";
    const auto invalid = [](const char* code) {
      return NativeMissionPreviewResult{
          422, {{"detail", code}, {"decision_only", true},
                {"mission_loaded", false}, {"motor_commands_enabled", false},
                {"spray_commands_enabled", false},
                {"physical_command_emitted", false}}};
    };

    if (!request.is_object()) return invalid(kInvalid);
    // Only preview input is permitted. Never accept forged authorization,
    // disarm, session or control-lease fields from untrusted JSON.
    for (auto it = request.begin(); it != request.end(); ++it) {
      const std::string key = it.key();
      if (key != "mission_run_id" && key != "mission_mode" &&
          key != "marking_points" && key != "extension_enabled" &&
          key != "extension_distance_m")
        return invalid("UNRECOGNIZED_OR_PRIVILEGED_FIELD");
    }

    try {
      if (!request.contains("mission_run_id") ||
          !request.at("mission_run_id").is_string())
        return invalid("INVALID_MISSION_RUN_ID");
      const std::string run_id = request.at("mission_run_id").get<std::string>();
      if (!dyx::rpp::valid_native_mission_run_id(run_id))
        return invalid("INVALID_MISSION_RUN_ID");

      const auto mode_json = request.value("mission_mode", nlohmann::json("automatic"));
      if (!mode_json.is_string()) return invalid("INVALID_MISSION_MODE");
      const std::string mode_text = mode_json.get<std::string>();
      if (mode_text != "automatic" && mode_text != "manual")
        return invalid("INVALID_MISSION_MODE");
      const auto mode = mode_text == "manual"
                            ? dyx::rpp::MissionMode::manual
                            : dyx::rpp::MissionMode::automatic;

      const auto extension_enabled_json =
          request.value("extension_enabled", nlohmann::json(false));
      if (!extension_enabled_json.is_boolean())
        return invalid("INVALID_EXTENSION_ENABLED");
      const bool extension_enabled = extension_enabled_json.get<bool>();

      const auto extension_distance_json =
          request.value("extension_distance_m", nlohmann::json(4.0));
      if (!extension_distance_json.is_number())
        return invalid("INVALID_EXTENSION_DISTANCE");
      const double extension_distance = extension_distance_json.get<double>();
      if (!std::isfinite(extension_distance) || extension_distance < 0.10 ||
          extension_distance > 20.0)
        return invalid("INVALID_EXTENSION_DISTANCE");

      if (!request.contains("marking_points") ||
          !request.at("marking_points").is_array())
        return invalid("INVALID_MARKING_POINTS");
      const auto& incoming = request.at("marking_points");
      if (incoming.size() < 2 || incoming.size() > 128)
        return invalid("INVALID_MARKING_POINT_COUNT");

      std::vector<dyx::rpp::Point2D> markings;
      markings.reserve(incoming.size());
      for (const auto& point : incoming) {
        if (!point.is_object() || point.size() != 2 ||
            !point.contains("x_m") || !point.contains("y_m") ||
            !point.at("x_m").is_number() || !point.at("y_m").is_number())
          return invalid("INVALID_MARKING_POINT_COORDINATES");
        const double x = point.at("x_m").get<double>();
        const double y = point.at("y_m").get<double>();
        if (!std::isfinite(x) || !std::isfinite(y))
          return invalid("INVALID_MARKING_POINT_COORDINATES");
        markings.push_back({x, y});
      }

      dyx::rpp::TrajectoryBuildConfig config;
      config.extension_enabled = extension_enabled;
      config.extension_distance_m = extension_distance;
      config.maximum_marking_points = 128;
      config.maximum_navigation_points = 20000;
      const auto trajectory = dyx::rpp::NativeTrajectoryBuilder(config).build(markings);
      const auto snapshot = dyx::rpp::signed_snapshot(trajectory.navigation, markings);
      const dyx::rpp::NativeMissionInstallation verified(snapshot, run_id, mode);

      if (verified.marking_goals.size() != markings.size())
        return invalid("GOAL_BINDING_MISMATCH");

      return {200, {{"ok", true}, {"state", "VALIDATED_PREVIEW_ONLY"},
                    {"coordinate_frame", "LOCAL_ENU_METRES"},
                    {"mission_run_id", run_id}, {"mission_mode", mode_text},
                    {"signature", snapshot.signature},
                    {"marking_count", markings.size()},
                    {"navigation_point_count", snapshot.navigation_points.size()},
                    {"dummy_alignment_points", trajectory.dummy_count},
                    {"extension_enabled", extension_enabled},
                    {"extension_distance_m", extension_distance},
                    {"mission_loaded", false}, {"mission_commands_enabled", false},
                    {"px4_disarm_verified", false}, {"control_lease_verified", false},
                    {"motor_commands_enabled", false},
                    {"spray_commands_enabled", false},
                    {"physical_command_emitted", false}}};
    } catch (const std::exception&) {
      return invalid("INVALID_MISSION_GEOMETRY");
    }
  }
};

}  // namespace dyx::http_backend
