#pragma once
// Deterministic policy for commissioning lifecycle checks.
// No ROS dependency, actuator output, or bypass of GPIO checks.
#include <cmath>
#include <cstdint>
namespace dyx::bridge {
struct LifecycleInput {
 bool armed, offboard, gpio_ok, px4_fresh_ok, sole_writer, latched;
 bool command_valid, command_fresh, stop_requested, awaiting_ack;
 bool ack_matches, ack_accepted, ack_timed_out, prestream_ready;
 double speed, yaw_rate;
};
struct LifecycleDecision {
 bool motion, zero_setpoint, stop_latch, request_disarm, accept_ack;
};
inline LifecycleDecision evaluate_lifecycle(const LifecycleInput& s) noexcept {
 const bool limits=std::isfinite(s.speed)&&std::isfinite(s.yaw_rate)&&
     std::fabs(s.speed)<=0.15&&std::fabs(s.yaw_rate)<=0.10;
 const bool guard=s.gpio_ok&&s.px4_fresh_ok&&s.sole_writer&&!s.latched;
 const bool bad_ack=s.awaiting_ack&&s.ack_matches&&!s.ack_accepted;
 const bool failed=s.stop_requested||s.ack_timed_out||bad_ack||
     (s.armed&&s.offboard&&(!guard||(!s.command_fresh&&s.command_valid)))||!limits;
 const bool motion=guard&&s.armed&&s.offboard&&s.command_valid&&s.command_fresh&&
     !s.stop_requested&&!s.ack_timed_out&&!bad_ack&&limits;
 return {motion,!motion,failed,failed&&s.armed,s.awaiting_ack&&s.ack_matches};
}
}
