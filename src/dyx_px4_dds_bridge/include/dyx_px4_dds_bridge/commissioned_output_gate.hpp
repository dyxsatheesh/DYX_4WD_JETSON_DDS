#pragma once
// Commissioned output *policy*; no ROS control message grants hardware authority.
// The live adapter must source evidence from an independently tested physical
// E-stop / deadman / RC takeover interface. No such adapter is installed yet.
#include <cmath>
namespace dyx::bridge {
struct CommissionedEvidence {
 bool independent_hardware_interlock_ok{false};
 bool operator_deadman_verified{false};
 bool rc_takeover_verified{false};
 bool estop_released_verified{false};
 bool estop_fresh{false};
 bool px4_status_fresh{false};
 bool px4_armed{false};
 bool px4_offboard{false};
 bool mission_authorized{false};
 bool mission_stop{true};
 bool command_fresh{false};
 bool heartbeat_fresh{false};
 bool estimator_valid{false};
 bool writer_quiet{false};
 bool requested_enable{false};
 double forward_mps{0};
 double yaw_rate_radps{0};
};
struct CommissionedDecision {
 bool allow_publish{false};
 const char* reason{"NOT_EVALUATED"};
};
inline CommissionedDecision evaluate_commissioned_output(const CommissionedEvidence& e) noexcept {
 if (!e.requested_enable) return {false,"OUTPUT_DISABLED"};
 if (!e.independent_hardware_interlock_ok) return {false,"PHYSICAL_INTERLOCK_UNVERIFIED"};
 if (!e.operator_deadman_verified) return {false,"DEADMAN_UNVERIFIED"};
 if (!e.rc_takeover_verified) return {false,"RC_TAKEOVER_UNVERIFIED"};
 if (!e.estop_fresh || !e.estop_released_verified) return {false,"ESTOP_UNVERIFIED"};
 if (!e.px4_status_fresh || !e.px4_armed || !e.px4_offboard) return {false,"PX4_NOT_READY"};
 if (e.mission_stop || !e.mission_authorized) return {false,"MISSION_STOP_OR_DENIED"};
 if (!e.command_fresh || !e.heartbeat_fresh || !e.estimator_valid) return {false,"STALE_OR_INVALID_INPUT"};
 if (!e.writer_quiet) return {false,"COMPETING_DDS_WRITER"};
 if (!std::isfinite(e.forward_mps) || !std::isfinite(e.yaw_rate_radps))
   return {false,"NONFINITE_COMMAND"};
 // Fixed commissioning limits; no parameter or mission request may widen them.
 if (std::fabs(e.forward_mps)>0.15 || std::fabs(e.yaw_rate_radps)>0.10)
   return {false,"WHEEL_LIFT_SPEED_LIMIT"};
 return {true,"COMMISSIONED_POLICY_SATISFIED"};
}
} // namespace dyx::bridge
