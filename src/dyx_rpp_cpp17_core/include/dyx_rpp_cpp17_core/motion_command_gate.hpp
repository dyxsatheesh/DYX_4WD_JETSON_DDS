#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace dyx::rpp {

// These booleans must come from the single trusted mission/safety
// owner. They are not authentication credentials or physical proof.
struct MotionSafetyEvidence {
  bool mission_running{false};
  bool safety_authorized{false};
  bool rtk_fixed{false};
  bool battery_ok{false};
  bool estimator_valid{false};
  bool obstacle_clear{false};
  bool operator_lease_valid{false};
  bool physical_estop_released{false};
  bool marking_reference_calibrated{false};
};

struct MotionDemand {
  double host_time_sec{0.0};
  std::uint64_t sequence{0};
  double forward_mps{0.0};
  double yaw_rate_enu_radps{0.0};
  double current_yaw_enu_rad{0.0};
  MotionSafetyEvidence safety{};
};

struct ValidatedMotionCommand {
  bool permitted{false};
  double velocity_north_mps{0.0};
  double velocity_east_mps{0.0};
  double yaw_rate_enu_radps{0.0};
};

class MotionCommandGate final {
 public:
  ValidatedMotionCommand evaluate(
      const MotionDemand& in) {

    if (latched_) return {};

    if (!std::isfinite(in.host_time_sec) ||
        !std::isfinite(in.forward_mps) ||
        !std::isfinite(in.yaw_rate_enu_radps) ||
        !std::isfinite(in.current_yaw_enu_rad) ||
        in.host_time_sec < 0.0 ||
        in.sequence == 0 ||
        in.forward_mps < 0.0 ||
        in.forward_mps > 1.0 ||
        std::abs(in.yaw_rate_enu_radps) > 0.18 ||
        std::abs(in.current_yaw_enu_rad) > 3.142 ||
        (last_time_ >= 0.0 &&
         (in.host_time_sec <= last_time_ ||
          in.host_time_sec - last_time_ > 0.10)) ||
        in.sequence <= last_sequence_) {
      latched_ = true;
      return {};
    }

    last_time_ = in.host_time_sec;
    last_sequence_ = in.sequence;

    const auto& s = in.safety;

    if (!s.mission_running ||
        !s.safety_authorized ||
        !s.rtk_fixed ||
        !s.battery_ok ||
        !s.estimator_valid ||
        !s.obstacle_clear ||
        !s.operator_lease_valid ||
        !s.physical_estop_released ||
        !s.marking_reference_calibrated) {
      latched_ = true;
      return {};
    }

    ValidatedMotionCommand out;
    out.permitted = true;
    out.velocity_north_mps =
        in.forward_mps * std::sin(in.current_yaw_enu_rad);
    out.velocity_east_mps =
        in.forward_mps * std::cos(in.current_yaw_enu_rad);
    out.yaw_rate_enu_radps = in.yaw_rate_enu_radps;

    return out;
  }

  bool latched() const { return latched_; }

  // An explicit verified, physically disarmed recovery is required.
  // The owning application must independently authenticate both facts.
  bool reset_for_new_run(
      bool trusted_authorization,
      bool verified_px4_disarmed) {
    if (!trusted_authorization || !verified_px4_disarmed)
      return false;

    last_time_ = -1.0;
    last_sequence_ = 0;
    latched_ = false;
    return true;
  }

 private:
  double last_time_{-1.0};
  std::uint64_t last_sequence_{0};
  bool latched_{true};  // No motion authorization at startup.
};

}  // namespace dyx::rpp
