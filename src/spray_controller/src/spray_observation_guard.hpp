#pragma once
#include "dyx_rpp_cpp17_core/spray_transaction.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace dyx::spray {
struct Observation {
  bool mission_stop{true};
  bool mission_status_fresh{false};
  bool actuator_writes_enabled{false};
  bool off_position_verified{false};
  int off_pwm_us{1200};
  int on_pwm_us{1400};
  double duration_sec{3.0};
};
inline void validate(const Observation& x) {
  if (x.off_pwm_us!=1200 || x.on_pwm_us!=1400 ||
      !std::isfinite(x.duration_sec) || x.duration_sec!=3.0 ||
      x.actuator_writes_enabled || x.off_position_verified)
    throw std::invalid_argument("UNCOMMISSIONED_AUX5_MUST_REMAIN_BLOCKED");
}
inline bool status_is_stop(const std::string& message) {
  // Exact status tokens; do not convert absence into permission.
  return message.find("emergency_stop=true")!=std::string::npos &&
         message.find("hold_zero=true")!=std::string::npos &&
         message.find("physical_command_emitted=false")!=std::string::npos;
}
inline dyx::rpp::SprayInput blocked_input(double now) {
  if (!std::isfinite(now) || now<0)throw std::invalid_argument("INVALID_TIME");
  dyx::rpp::SprayInput in{};
  in.now_sec=now;
  in.request_spray=false;
  in.controller_enabled=false;
  in.physical_estop_released=false;
  in.mission_enabled=false;
  in.mission_spray_hold=false;
  in.safety_motion_hold_verified=false;
  in.raw_rtk_fixed_survey_pass=false;
  in.px4_armed_offboard=false;
  return in;
}
}
