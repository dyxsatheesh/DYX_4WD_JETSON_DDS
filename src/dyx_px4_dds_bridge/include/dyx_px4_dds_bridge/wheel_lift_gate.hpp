#pragma once
// DYX precommissioning gate: verifies evidence but NEVER authorizes physical output.
// A positive preview is not an actuation token.
#include <cmath>
namespace dyx::bridge {
struct WheelLiftEvidence {
 bool operator_present{false};
 bool mechanical_wheels_lifted_confirmed{false};
 bool physical_estop_released_verified{false};
 bool rc_takeover_verified{false};
 bool px4_armed{false};
 bool offboard_active{false};
 bool px4_status_fresh{false};
 bool command_fresh{false};
 bool heartbeat_fresh{false};
 bool estop_signal_fresh{false};
 bool estop_asserted{true};
 bool writer_graph_quiet{false};
 bool output_path_certified{false};
 double speed_mps{0.0};
 double yaw_rate_radps{0.0};
};
struct WheelLiftDecision {
 bool prerequisites_met{false};
 bool publish_authorized{false};
 const char* reason{"NOT_EVALUATED"};
};
inline WheelLiftDecision inspect_wheel_lift(const WheelLiftEvidence& e) noexcept {
 if (!e.operator_present) return {false,false,"OPERATOR_ABSENT"};
 if (!e.mechanical_wheels_lifted_confirmed) return {false,false,"WHEELS_NOT_CONFIRMED_LIFTED"};
 if (!e.physical_estop_released_verified || !e.estop_signal_fresh || e.estop_asserted)
   return {false,false,"ESTOP_NOT_VERIFIED_RELEASED"};
 if (!e.rc_takeover_verified) return {false,false,"RC_TAKEOVER_NOT_VERIFIED"};
 if (!e.px4_status_fresh || !e.px4_armed || !e.offboard_active)
   return {false,false,"PX4_STATE_NOT_READY"};
 if (!e.command_fresh || !e.heartbeat_fresh) return {false,false,"STALE_COMMAND_OR_HEARTBEAT"};
 if (!e.writer_graph_quiet) return {false,false,"OTHER_WRITER_OR_UNKNOWN"};
 if (!std::isfinite(e.speed_mps) || !std::isfinite(e.yaw_rate_radps) ||
     std::fabs(e.speed_mps)>0.15 || std::fabs(e.yaw_rate_radps)>0.10)
   return {false,false,"COMMISSIONING_LIMIT_EXCEEDED"};
 if (!e.output_path_certified) return {false,false,"OUTPUT_PATH_NOT_CERTIFIED"};
 // Physical actuation is always denied until a separately reviewed,
 // externally authorized controller is integrated and tested.
 return {true,false,"BENCH_PREVIEW_ONLY_NO_ACTUATION"};
}
} // namespace dyx::bridge
