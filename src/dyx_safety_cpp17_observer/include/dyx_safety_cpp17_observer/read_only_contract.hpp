#pragma once
// H6D: observation-only. Never a motor, servo, Offboard or mission permission.
#include <cmath>
#include <string>
namespace dyx::h6d {
struct Observation {
  bool physical_command_permitted{false};
  bool mission_transition_permitted{false};
  bool spray_permitted{false};
  bool correction_source_verified{false};
  bool obstacle_source_verified{false};
  bool telemetry_integrity_fault{false};
  bool battery_critical_latched{false};
  bool rtk_float_confirming{false};
  bool battery_debounce_pending{false};
  std::string reason{"UNINITIALIZED"};
};
// Even when a hypothetical safety model reports READY, these source gaps
// prohibit converting its output into physical motion permission.
inline Observation observe_only(bool telemetry_fault,
                                bool battery_latched,
                                bool float_confirming,
                                bool battery_pending,
                                const char* model_reason) {
  Observation o;
  o.telemetry_integrity_fault=telemetry_fault;
  o.battery_critical_latched=battery_latched;
  o.rtk_float_confirming=float_confirming;
  o.battery_debounce_pending=battery_pending;
  o.reason = telemetry_fault ? "TELEMETRY_INTEGRITY_LATCHED" :
      battery_latched ? "BATTERY_LOCKOUT_OBSERVED" :
      "MISSING_TRUSTED_RTCM_AND_OBSTACLE_SOURCES";
  if (model_reason == nullptr || model_reason[0] == '\0')
    o.reason = "SAFETY_MODEL_REASON_MISSING";
  return o;
}
} // namespace dyx::h6d
