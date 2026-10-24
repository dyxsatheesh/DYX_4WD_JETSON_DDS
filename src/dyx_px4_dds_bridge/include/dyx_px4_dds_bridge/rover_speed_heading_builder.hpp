#pragma once
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/rover_speed_setpoint.hpp>
#include <px4_msgs/msg/rover_attitude_setpoint.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
namespace dyx::bridge {
struct PreparedSpeedHeading {
 px4_msgs::msg::OffboardControlMode mode{};
 px4_msgs::msg::RoverSpeedSetpoint speed{};
 px4_msgs::msg::RoverAttitudeSetpoint attitude{};
 bool valid{false};
 bool publish_authorized{false}; // Requires independent commissioning authority.
};
inline PreparedSpeedHeading prepare_rover_speed_heading(
 double forward_mps, double heading_enu_rad,
 std::uint64_t timestamp_us, bool inputs_valid) noexcept {
 PreparedSpeedHeading o{};
 if (!inputs_valid || timestamp_us==0 || !std::isfinite(forward_mps) ||
     !std::isfinite(heading_enu_rad) || std::abs(forward_mps)>1.0) return o;
 o.mode.timestamp=timestamp_us;
 o.mode.velocity=true;
 o.speed.timestamp=timestamp_us;
 o.speed.speed_body_x=static_cast<float>(forward_mps);
 o.speed.speed_body_y=std::numeric_limits<float>::quiet_NaN();
 o.attitude.timestamp=timestamp_us;
 // ENU yaw: 0 East, +pi/2 North. PX4 NED: 0 North, +pi/2 East.
 o.attitude.yaw_setpoint=static_cast<float>(
     std::remainder(1.57079632679489661923-heading_enu_rad,
                    6.28318530717958647692));
 o.valid=true;
 return o;
}
} // namespace dyx::bridge
