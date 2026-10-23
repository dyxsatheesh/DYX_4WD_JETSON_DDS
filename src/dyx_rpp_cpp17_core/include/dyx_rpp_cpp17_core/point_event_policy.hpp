#pragma once
// C++17 port of the exercised Python point_event_policy.py functionality.
// Pure semantic policy: NEVER authorizes motion or advances mission state.
#include <algorithm>
#include <cctype>
#include <cmath>
#include <optional>
#include <string>

namespace dyx::rpp::point_event {
inline std::string normalize(std::string event) {
  const auto a = event.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) return {};
  const auto b = event.find_last_not_of(" \t\r\n");
  event = event.substr(a, b-a+1);
  std::transform(event.begin(), event.end(), event.begin(),
     [](unsigned char c){return static_cast<char>(std::toupper(c));});
  return event;
}
inline bool release_first_marking(const std::string &event, int point_index) {
  if (point_index != 0) return false;
  const auto s=normalize(event);
  return s=="COMPLETED" || s=="FAILED" || s=="ACCURACY_FAILED" || s=="SKIPPED";
}
inline bool first_marking_approach_active(bool first_marking_resolved,
    bool c_line_locked, bool c_line_bearing_available,
    bool has_marking_waypoints, int segment_goal_number) {
  return !first_marking_resolved && segment_goal_number==1 &&
    c_line_locked && c_line_bearing_available && has_marking_waypoints;
}
enum class LatchedStopOutcome { none, captured, missed };
inline LatchedStopOutcome latched_stop_terminal_outcome(double target_distance_m,
    double current_speed_mps, double tolerance_m, double stationary_speed_mps) {
  if (!std::isfinite(target_distance_m) || !std::isfinite(current_speed_mps) ||
      !std::isfinite(tolerance_m) || !std::isfinite(stationary_speed_mps) ||
      target_distance_m < 0.0 || current_speed_mps < 0.0 ||
      tolerance_m <= 0.0 || stationary_speed_mps < 0.0 ||
      current_speed_mps > stationary_speed_mps) return LatchedStopOutcome::none;
  return target_distance_m <= tolerance_m ? LatchedStopOutcome::captured
                                          : LatchedStopOutcome::missed;
}
} // namespace dyx::rpp::point_event
