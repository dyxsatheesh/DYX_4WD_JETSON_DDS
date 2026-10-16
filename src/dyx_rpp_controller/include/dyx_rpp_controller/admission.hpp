#pragma once
// ROS-free, NEVER an actuation permission. Validates a preview-only execution tick.
#include <cmath>
#include <cstdint>
#include <string>
namespace dyx::preview {
struct Evidence {
  double now_s{0.0};
  bool preview_enabled{false};
  bool position_seen{false}, rate_seen{false}, goal_seen{false};
  bool estimator_valid{false}, telemetry_finite{false}, reset_detected{false}, invalid_goal{false};
  double position_age_s{0.0}, rate_age_s{0.0}, goal_age_s{0.0};
  double goal_north_m{0.0}, goal_east_m{0.0};
  std::uint64_t position_sample_us{0}, rate_sample_us{0};
};
struct Decision {
  bool preview_permitted{false};
  bool physical_command_permitted{false}; // invariant: always false
  bool fault_latched{false};
  const char* reason{"UNINITIALIZED"};
};
class Admission final {
 public:
  Decision inspect(const Evidence& e) {
    // Activation is a launch-time preview flag; even an enabled preview is NOT motion.
    if (!e.preview_enabled) return {false, false, latched_, "PREVIEW_DISABLED"};
    if (latched_) return {false, false, true, "FAULT_LATCHED_RESTART_REQUIRED"};
    if (!std::isfinite(e.now_s) || e.now_s < 0.0 ||
        (last_time_s_ >= 0.0 && e.now_s < last_time_s_))
      return latch("CLOCK_REGRESSION_OR_INVALID");
    last_time_s_ = e.now_s;
    if (e.reset_detected) return latch("PX4_ESTIMATOR_RESET");
    if (e.invalid_goal) return latch("INVALID_GOAL");
    if (!e.position_seen || !e.rate_seen || !e.goal_seen) {
      if (ever_ready_) return latch("TELEMETRY_OR_GOAL_DISAPPEARED");
      return {false, false, false, "WAITING_FOR_INPUTS"};
    }
    if (!e.estimator_valid || !e.telemetry_finite) return latch("INVALID_EKF_OR_NONFINITE_TELEMETRY");
    if (!std::isfinite(e.position_age_s) || !std::isfinite(e.rate_age_s) ||
        !std::isfinite(e.goal_age_s) ||
        e.position_age_s < 0.0 || e.rate_age_s < 0.0 || e.goal_age_s < 0.0 ||
        e.position_age_s > 0.25 || e.rate_age_s > 0.25 || e.goal_age_s > 2.0)
      return latch("STALE_INPUT");
    if (!std::isfinite(e.goal_north_m) || !std::isfinite(e.goal_east_m) ||
        std::abs(e.goal_north_m) > 100000.0 || std::abs(e.goal_east_m) > 100000.0)
      return latch("NONFINITE_OR_OUT_OF_RANGE_GOAL");
    if (!e.position_sample_us || !e.rate_sample_us)
      return latch("MISSING_SAMPLE_TIME");
    const auto skew = e.position_sample_us >= e.rate_sample_us
      ? e.position_sample_us - e.rate_sample_us
      : e.rate_sample_us - e.position_sample_us;
    if (skew > 100000) return latch("TELEMETRY_TIME_SKEW");
    ever_ready_ = true;
    return {true, false, false, "PREVIEW_ONLY"};
  }
  bool fault_latched() const { return latched_; }
 private:
  Decision latch(const char* why) {
    latched_ = true;
    return {false, false, true, why};
  }
  double last_time_s_{-1.0};
  bool latched_{false}, ever_ready_{false};
};
} // namespace dyx::preview
