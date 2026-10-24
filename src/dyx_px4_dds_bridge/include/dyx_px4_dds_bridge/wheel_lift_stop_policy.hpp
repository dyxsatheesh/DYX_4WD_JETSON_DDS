#pragma once
// Pure production stop policy: independently unit-tested, no ROS dependencies.
namespace dyx::bridge {
struct WheelLiftStopInputs {
 bool had_prestream;
 bool gpio_ok;
 bool px4_ok;
 bool sole_writer;
 bool armed;
 bool was_offboard;
 bool command_valid;
 bool command_fresh;
};
struct WheelLiftStopDecision {bool latch; bool request_disarm;};
inline WheelLiftStopDecision wheel_lift_stop_policy(WheelLiftStopInputs e) noexcept {
 const bool lost_guard = e.had_prestream && (!e.gpio_ok || !e.px4_ok || !e.sole_writer);
 const bool timed_out = e.was_offboard && e.command_valid && !e.command_fresh;
 return {lost_guard || timed_out, (lost_guard && e.armed) || timed_out};
}
}
