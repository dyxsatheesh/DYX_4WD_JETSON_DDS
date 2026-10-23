#pragma once
// Port of archived Python marking_arrival_policy.py: pure policy, not motion authority.
#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

namespace dyx::mission::arrival {
enum class PhaseA { START_VERIFICATION, FAIL_NOW, KEEP_APPROACHING };
enum class FailureAction { AUTO_CONTINUE, WAITING_FOR_NEXT };

inline std::string uppercase_trimmed(std::string s) {
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = s.find_last_not_of(" \t\r\n");
  s = s.substr(first, last - first + 1);
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
}

inline PhaseA phase_a_decision(bool inside_tolerance, bool stationary,
                               const std::string& rpp_outcome = "",
                               bool past_goal_plane = false) {
  if (inside_tolerance && stationary) return PhaseA::START_VERIFICATION;
  const auto result = uppercase_trimmed(rpp_outcome);
  const bool missed = result == "MISSED" || result == "CAPTURED";
  if (stationary && !inside_tolerance && (missed || past_goal_plane))
    return PhaseA::FAIL_NOW;
  return PhaseA::KEEP_APPROACHING;
}

inline FailureAction after_fail_mode_action(const std::string& mode) {
  return uppercase_trimmed(mode) == "MANUAL"
             ? FailureAction::WAITING_FOR_NEXT : FailureAction::AUTO_CONTINUE;
}

inline bool should_publish_rpp_captured(double target_distance_m,
                                        double speed_mps,
                                        double waypoint_tolerance_m,
                                        double stationary_speed_mps) {
  return std::isfinite(target_distance_m) && std::isfinite(speed_mps) &&
         std::isfinite(waypoint_tolerance_m) &&
         std::isfinite(stationary_speed_mps) &&
         target_distance_m >= 0.0 && speed_mps >= 0.0 &&
         waypoint_tolerance_m > 0.0 && stationary_speed_mps >= 0.0 &&
         target_distance_m <= waypoint_tolerance_m &&
         speed_mps <= stationary_speed_mps;
}
}  // namespace dyx::mission::arrival
