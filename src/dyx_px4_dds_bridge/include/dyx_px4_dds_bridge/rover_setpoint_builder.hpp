#pragma once

#include "dyx_px4_dds_bridge/native_rover_dds_contract.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/rover_speed_setpoint.hpp>
#include <px4_msgs/msg/rover_rate_setpoint.hpp>

#include <cstdint>
#include <cmath>
#include <limits>

namespace dyx::bridge {

struct PreparedRoverSetpoints {
  px4_msgs::msg::OffboardControlMode mode{};
  px4_msgs::msg::RoverSpeedSetpoint speed{};
  px4_msgs::msg::RoverRateSetpoint rate{};
  bool valid{false};
  bool publish_authorized{false};
};

inline PreparedRoverSetpoints prepare_rover_setpoints(
    const RoverSpeedRateWire& wire,
    std::uint64_t timestamp_us,
    bool command_fresh) noexcept {

  PreparedRoverSetpoints out{};

  if (!command_fresh || !wire.valid || timestamp_us == 0 ||
      !std::isfinite(wire.speed_body_x) ||
      !std::isfinite(wire.yaw_rate_setpoint)) {
    return out;
  }

  out.mode.timestamp = timestamp_us;
  out.mode.position = false;
  out.mode.velocity = true;
  out.mode.acceleration = false;
  out.mode.attitude = false;
  out.mode.body_rate = false;
  out.mode.thrust_and_torque = false;
  out.mode.direct_actuator = false;

  out.speed.timestamp = timestamp_us;
  out.speed.speed_body_x = wire.speed_body_x;
  out.speed.speed_body_y =
      std::numeric_limits<float>::quiet_NaN();

  out.rate.timestamp = timestamp_us;
  out.rate.yaw_rate_setpoint = wire.yaw_rate_setpoint;

  out.valid = true;

  // Only message construction is implemented.
  // This never grants mission or hardware authority.
  out.publish_authorized = false;

  return out;
}

}  // namespace dyx::bridge
