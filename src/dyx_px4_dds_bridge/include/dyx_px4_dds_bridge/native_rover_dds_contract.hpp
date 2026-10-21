#pragma once
// D3B: PX4 1.17 rover native speed+rate wire contract (NON-ACTUATING).
// Data-only adapter. No ROS publisher; cannot grant PX4 or motion authority.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace dyx::bridge {
struct RoverSpeedRateWire {
  float speed_body_x{0.0f};
  float speed_body_y{std::numeric_limits<float>::quiet_NaN()};
  float yaw_rate_setpoint{0.0f};
  float higher_priority_attitude_yaw{std::numeric_limits<float>::quiet_NaN()};
  bool velocity_control_flag{true};
  bool valid{false};
  bool publish_authorized{false}; // D3B never authorizes publication.
};

// Sign conventions: body-forward speed +forward, -reverse; ENU yaw CCW,
// PX4 RoverRateSetpoint yaw_rate_setpoint NED clockwise positive.
inline RoverSpeedRateWire rover_speed_rate_wire(double signed_body_speed_mps,
                                                double yaw_rate_enu_radps,
                                                bool measurements_valid,
                                                double max_abs_speed_mps=1.0,
                                                double max_abs_yaw_rate_radps=0.18) noexcept {
  RoverSpeedRateWire o{};
  if (!measurements_valid || !std::isfinite(signed_body_speed_mps) ||
      !std::isfinite(yaw_rate_enu_radps) || !std::isfinite(max_abs_speed_mps) ||
      !std::isfinite(max_abs_yaw_rate_radps) || max_abs_speed_mps<=0.0 ||
      max_abs_speed_mps>1.0 || max_abs_yaw_rate_radps<=0.0 || max_abs_yaw_rate_radps>0.18)
    return o;
  o.speed_body_x = static_cast<float>(std::clamp(signed_body_speed_mps,
      -max_abs_speed_mps, max_abs_speed_mps));
  o.yaw_rate_setpoint = static_cast<float>(-std::clamp(yaw_rate_enu_radps,
      -max_abs_yaw_rate_radps, max_abs_yaw_rate_radps));
  o.valid=true;
  return o;
}
} // namespace dyx::bridge
