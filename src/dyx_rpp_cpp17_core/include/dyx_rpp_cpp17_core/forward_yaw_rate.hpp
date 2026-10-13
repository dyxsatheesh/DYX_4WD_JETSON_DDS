#pragma once
// Phase 2E3: NON-ACTUATING forward-only velocity + signed yaw-rate planning.
// ENU yaw: CCW from East. PX4 trajectory_setpoint: NED (North/East), yaw CW.
// The firmware projects velocity onto current heading to obtain body-X speed.
// Always build N/E velocity from CURRENT yaw, not desired yaw, to avoid reverse.
// This class does not publish to PX4, arm, change mode, or command motor drivers.

#include "dyx_rpp_cpp17_core/primitives.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace dyx::rpp {

enum class ForwardYawPhase { TRACK, WAIT_PIVOT, PIVOT, SETTLE, SAFE_HOLD };
inline const char *phase_name(ForwardYawPhase p) {
  switch (p) {
    case ForwardYawPhase::TRACK: return "TRACK";
    case ForwardYawPhase::WAIT_PIVOT: return "WAIT_PIVOT";
    case ForwardYawPhase::PIVOT: return "PIVOT";
    case ForwardYawPhase::SETTLE: return "SETTLE";
    case ForwardYawPhase::SAFE_HOLD: return "SAFE_HOLD";
  }
  return "SAFE_HOLD";
}

struct ForwardYawConfig {
  double speed_max_mps{1.0};
  double moving_kp{0.85};
  double pivot_kp{1.2};
  double moving_yaw_rate_max_radps{0.15};
  double pivot_yaw_rate_max_radps{0.18};
  double moving_deadband_rad{0.5 * pi / 180.0};
  double pivot_enter_rad{15.0 * pi / 180.0};
  double pivot_release_rad{2.0 * pi / 180.0};
  double pivot_rate_settle_radps{0.05};
  double pivot_settle_sec{0.20};
  double max_dt_sec{0.10};
  double pivot_timeout_sec{10.0};
  // Any change to requested pivot heading during a maneuver fails closed.
  double pivot_target_change_max_rad{0.5 * pi / 180.0};
  void validate() const {
    for (double v: {speed_max_mps, moving_kp, pivot_kp,
       moving_yaw_rate_max_radps, pivot_yaw_rate_max_radps, pivot_enter_rad,
       pivot_release_rad, pivot_rate_settle_radps, pivot_settle_sec,
       max_dt_sec, pivot_timeout_sec, pivot_target_change_max_rad}) {
      if (!std::isfinite(v) || v <= 0.0)
        throw std::invalid_argument("forward yaw config must be finite, positive");
    }
    if (!std::isfinite(moving_deadband_rad) || moving_deadband_rad < 0.0 ||
        moving_deadband_rad >= pivot_release_rad ||
        pivot_release_rad >= pivot_enter_rad || pivot_enter_rad >= pi ||
        pivot_target_change_max_rad >= pivot_release_rad ||
        speed_max_mps > 1.0 ||
        moving_yaw_rate_max_radps > 0.18 ||
        pivot_yaw_rate_max_radps > 0.18 ||
        pivot_settle_sec >= pivot_timeout_sec)
      throw std::invalid_argument("unsafe forward yaw config thresholds");
  }
};

struct ForwardYawInput {
  double now_sec{0.0};
  double dt_sec{0.0};
  double rover_yaw_enu_rad{0.0};
  double target_yaw_enu_rad{0.0};
  double measured_yaw_rate_enu_radps{0.0};
  double requested_forward_speed_mps{0.0};
  bool motion_enabled{false};
  bool telemetry_fresh{false};
  bool pivot_permission{false};
  bool stop_requested{false};
};

struct ForwardYawOutput {
  ForwardYawPhase phase{ForwardYawPhase::SAFE_HOLD};
  double forward_speed_mps{0.0};
  double velocity_north_mps{0.0};
  double velocity_east_mps{0.0};
  double yaw_rate_enu_radps{0.0};
  double yaw_rate_ned_radps{0.0};
  double heading_error_rad{0.0};
  bool safe_hold{true};
  bool pivot_complete{false};
  std::string reason{"UNINITIALIZED"};
};

class ForwardYawRatePlanner {
 public:
  explicit ForwardYawRatePlanner(ForwardYawConfig c = {}) : config_(c) {
    config_.validate();
  }

  // Reset only at an explicit, externally approved mission/control boundary.
  // Never auto-clear a SAFE_HOLD on telemetry resumption.
  void reset() {
    phase_ = ForwardYawPhase::TRACK;
    last_time_ = -1.0;
    pivot_start_ = -1.0;
    pivot_target_ = 0.0;
    settle_dwell_ = 0.0;
    reason_ = "RESET";
  }

  ForwardYawOutput step(const ForwardYawInput &s) {
    if (phase_ == ForwardYawPhase::SAFE_HOLD)
      return zero("SAFE_HOLD_LATCHED");
    for (double v: {s.now_sec,s.dt_sec,s.rover_yaw_enu_rad,
        s.target_yaw_enu_rad,s.measured_yaw_rate_enu_radps,
        s.requested_forward_speed_mps}) {
      if (!std::isfinite(v)) return latch("NONFINITE_INPUT");
    }
    if (s.now_sec < 0.0 || s.dt_sec < 0.0 ||
        s.requested_forward_speed_mps < 0.0 ||
        (last_time_ >= 0.0 && s.now_sec < last_time_))
      return latch("INVALID_OR_REVERSE_INPUT");
    const double elapsed_since_previous = last_time_ < 0.0 ? 0.0 :
      std::max(0.0, s.now_sec - last_time_);
    last_time_ = s.now_sec;
    if (!s.motion_enabled || !s.telemetry_fresh) return latch("SAFETY_GATE");
    if (s.stop_requested) {
      // Deliberate no-motion HOLD (e.g. completed terminal stop).
      // Do not interpret a zero-speed hold as permission for any pivot.
      return zero("STOP_REQUESTED");
    }

    const double err = wrap_angle(s.target_yaw_enu_rad - s.rover_yaw_enu_rad);
    const double abs_err = std::abs(err);
    if (phase_ == ForwardYawPhase::PIVOT || phase_ == ForwardYawPhase::SETTLE) {
      if (std::abs(wrap_angle(s.target_yaw_enu_rad - pivot_target_)) >
          config_.pivot_target_change_max_rad)
        return latch("PIVOT_TARGET_CHANGED");
      if (s.now_sec - pivot_start_ >= config_.pivot_timeout_sec)
        return latch("PIVOT_TIMEOUT");
      if (abs_err <= config_.pivot_release_rad &&
          std::abs(s.measured_yaw_rate_enu_radps) <= config_.pivot_rate_settle_radps) {
        phase_ = ForwardYawPhase::SETTLE;
        // Duplicate position timestamps cannot manufacture settling evidence.
        settle_dwell_ += std::min({s.dt_sec, elapsed_since_previous,
                                   config_.max_dt_sec});
        if (settle_dwell_ + 1e-9 >= config_.pivot_settle_sec) {
          phase_ = ForwardYawPhase::TRACK;
          settle_dwell_ = 0.0;
          auto o = zero("PIVOT_COMPLETE_ZERO", err);
          o.pivot_complete = true;
          return o;  // no movement in release tick
        }
        return zero("PIVOT_SETTLING", err);
      }
      phase_ = ForwardYawPhase::PIVOT;
      settle_dwell_ = 0.0;
      const double rate = rate_for(err, config_.pivot_kp,
                                   config_.pivot_yaw_rate_max_radps);
      return output(0.0, s.rover_yaw_enu_rad, rate, err, "PIVOTING");
    }

    if (abs_err >= config_.pivot_enter_rad) {
      if (!s.pivot_permission) {
        phase_ = ForwardYawPhase::WAIT_PIVOT;
        return zero("PIVOT_PERMISSION_REQUIRED", err);
      }
      phase_ = ForwardYawPhase::PIVOT;
      pivot_start_ = s.now_sec;
      pivot_target_ = wrap_angle(s.target_yaw_enu_rad);
      settle_dwell_ = 0.0;
      return output(0.0, s.rover_yaw_enu_rad,
                    rate_for(err,config_.pivot_kp,config_.pivot_yaw_rate_max_radps),
                    err,"PIVOTING");
    }

    // Normal forward tracking: heading correction is by yaw-rate only.
    // Velocity remains aligned to CURRENT rover heading, not target bearing.
    phase_ = ForwardYawPhase::TRACK;
    const double rate = abs_err <= config_.moving_deadband_rad ? 0.0 :
      rate_for(err, config_.moving_kp, config_.moving_yaw_rate_max_radps);
    return output(std::min(s.requested_forward_speed_mps,config_.speed_max_mps),
                  s.rover_yaw_enu_rad, rate, err, "FORWARD_TRACK");
  }

 private:
  static double rate_for(double err, double kp, double limit) {
    return std::clamp(kp * err, -limit, limit);
  }
  ForwardYawOutput output(double forward, double rover_yaw, double rate,
                          double error, const char *reason) const {
    // ENU yaw 0=East; NED velocity ordering: North, East.
    const double north = forward * std::sin(rover_yaw);
    const double east = forward * std::cos(rover_yaw);
    // In PX4 firmware combined mode body-X = N*cos(yaw_NED) + E*sin(yaw_NED)
    // equals forward by construction, so no reverse motion request is possible.
    return {phase_,forward,north,east,rate,-rate,error,false,false,reason};
  }
  ForwardYawOutput zero(const char *reason, double error = 0.0) const {
    return {phase_,0.0,0.0,0.0,0.0,0.0,error,
            phase_==ForwardYawPhase::SAFE_HOLD,false,reason};
  }
  ForwardYawOutput latch(const char *reason) {
    phase_ = ForwardYawPhase::SAFE_HOLD;
    reason_ = reason;
    return zero(reason_.c_str());
  }
  ForwardYawConfig config_{};
  ForwardYawPhase phase_{ForwardYawPhase::TRACK};
  double last_time_{-1.0};
  double pivot_start_{-1.0};
  double pivot_target_{0.0};
  double settle_dwell_{0.0};
  std::string reason_{"RESET"};
};

}  // namespace dyx::rpp
