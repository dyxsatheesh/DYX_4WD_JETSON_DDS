#pragma once
#include <cmath>
#include <cstdint>

namespace dyx::http_backend {
// Diagnostic-only assessment, NEVER an authorization capability.
struct PositiveCommandEvidence {
 bool mission_loaded{false};
 bool stop_latched{true};
 bool telemetry_fault{true};
 bool rtk_fixed{false};
 bool receiver_age_trusted{false};
 bool position_fresh{false};
 bool rear_reference_calibrated{false};
 bool battery_reserve_verified{false};
 bool obstacle_clear_verified{false};
 bool physical_estop_clear_verified{false};
 bool operator_lease_verified{false};
 bool px4_command_path_commissioned{false};
 bool spray_safety_commissioned{false};
 bool timing_fault{true};
};
struct PositiveCommandPreflight {
 std::uint32_t missing_gate_count{0};
 const char* first_blocker{"UNKNOWN"};
 bool diagnostic_preconditions_met{false};
 bool motion_authorized{false}; // Permanently false in this version.
};
inline PositiveCommandPreflight assess_positive_command_preflight(const PositiveCommandEvidence& e) {
 PositiveCommandPreflight out{};
 const auto check=[&out](bool ok,const char* name){
  if (!ok) { if(out.missing_gate_count==0)out.first_blocker=name; ++out.missing_gate_count; }
 };
 check(e.mission_loaded,"NO_MISSION");
 check(!e.stop_latched,"STOP_LATCHED");
 check(!e.telemetry_fault,"TELEMETRY_FAULT");
 check(e.rtk_fixed,"RTK_NOT_FIXED");
 check(e.receiver_age_trusted,"RECEIVER_AGE_NOT_TRUSTED");
 check(e.position_fresh,"POSITION_STALE");
 check(e.rear_reference_calibrated,"REAR_REFERENCE_UNCALIBRATED");
 check(e.battery_reserve_verified,"BATTERY_RESERVE_UNVERIFIED");
 check(e.obstacle_clear_verified,"OBSTACLE_UNVERIFIED");
 check(e.physical_estop_clear_verified,"ESTOP_UNVERIFIED");
 check(e.operator_lease_verified,"OPERATOR_LEASE_UNVERIFIED");
 check(e.px4_command_path_commissioned,"PX4_CONTROL_UNCOMMISSIONED");
 check(e.spray_safety_commissioned,"SPRAY_UNCOMMISSIONED");
 check(!e.timing_fault,"CONTROL_TIMING_FAULT");
 out.diagnostic_preconditions_met=out.missing_gate_count==0;
 if(out.diagnostic_preconditions_met)out.first_blocker="NONE_DIAGNOSTIC_ONLY";
 return out;
}
}
