#pragma once
// Phase 2D2: pure C++17 port of Python terminal_stop_regulator.py.
// Non-actuating. All coordinates/evidence are in the original estimator frame;
// certification is NOT independent real-world survey proof.
#include "dyx_rpp_cpp17_core/stationary_detector.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace dyx::rpp {

enum class RadialStopState { track, terminal_guidance, brake_profile,
  zero_latch, settle, certified, hold_zero, hold_fail };
enum class MotionDirection { forward, zero };
enum class RadialStopFailure { none, invalid_measurement, stale_telemetry,
  terminal_request_lost, identity_changed, terminal_timeout, settle_timeout,
  settled_outside_tolerance };

inline const char *to_string(RadialStopState s) {
  switch (s) {
    case RadialStopState::track: return "track";
    case RadialStopState::terminal_guidance: return "terminal_guidance";
    case RadialStopState::brake_profile: return "brake_profile";
    case RadialStopState::zero_latch: return "zero_latch";
    case RadialStopState::settle: return "settle";
    case RadialStopState::certified: return "certified";
    case RadialStopState::hold_zero: return "hold_zero";
    case RadialStopState::hold_fail: return "hold_fail";
  }
  return "unknown";
}
inline const char *to_string(MotionDirection d) {
  return d == MotionDirection::forward ? "forward" : "zero";
}
inline const char *to_string(RadialStopFailure f) {
  switch (f) {
    case RadialStopFailure::none: return "none";
    case RadialStopFailure::invalid_measurement: return "invalid_measurement";
    case RadialStopFailure::stale_telemetry: return "stale_telemetry";
    case RadialStopFailure::terminal_request_lost: return "terminal_request_lost";
    case RadialStopFailure::identity_changed: return "identity_changed";
    case RadialStopFailure::terminal_timeout: return "terminal_timeout";
    case RadialStopFailure::settle_timeout: return "settle_timeout";
    case RadialStopFailure::settled_outside_tolerance: return "settled_outside_tolerance";
  }
  return "unknown";
}

struct RadialStopConfig {
  double radial_tolerance_m{0.020};
  double terminal_guidance_distance_m{0.75};
  double conservative_decel_mps2{0.30};
  double brake_margin_m{0.010};
  double stationary_window_sec{0.50};
  double stationary_displacement_m{0.005};
  double stationary_yaw_rate_radps{0.050};
  double maximum_position_sample_gap_sec{0.20};
  double terminal_timeout_sec{15.0};
  double settle_timeout_sec{5.0};

  void validate() const {
    for (double v : {radial_tolerance_m, terminal_guidance_distance_m,
         conservative_decel_mps2, stationary_window_sec,
         stationary_displacement_m, stationary_yaw_rate_radps,
         maximum_position_sample_gap_sec, terminal_timeout_sec,
         settle_timeout_sec}) {
      if (!std::isfinite(v) || v <= 0.0)
        throw std::invalid_argument("radial-stop positive configuration invalid");
    }
    if (!std::isfinite(brake_margin_m) || brake_margin_m < 0.0 ||
        brake_margin_m > radial_tolerance_m ||
        terminal_guidance_distance_m <= radial_tolerance_m ||
        maximum_position_sample_gap_sec >= stationary_window_sec ||
        settle_timeout_sec <= stationary_window_sec ||
        terminal_timeout_sec <= settle_timeout_sec)
      throw std::invalid_argument("radial-stop configuration thresholds invalid");
  }
};

struct RadialStopInput {
  double monotonic_time_sec{};
  double position_sample_time_sec{};
  bool active{};
  std::optional<std::string> terminal_identity{};
  double along_remaining_m{};
  double cross_error_m{};
  double position_x_m{};
  double position_y_m{};
  double position_derived_speed_mps{};
  double measured_yaw_rate_radps{};
  double tracking_speed_command_mps{};
  bool telemetry_fresh{};
};

struct RadialStopCertificate {
  int version{2};
  std::string terminal_identity{};
  double certified_timestamp_sec{};
  double radial_error_m{};
  double along_error_m{};
  double cross_error_m{};
  double final_position_x_m{};
  double final_position_y_m{};
  double stationary_window_sec{};
  double maximum_stationary_displacement_m{};
  double maximum_abs_yaw_rate_radps{};
  std::string speed_source{"position_derived"};
  std::string truth_frame{"px4_estimator_frame_only"};
};

struct RadialStopOutput {
  RadialStopState previous_state{RadialStopState::track};
  RadialStopState state{RadialStopState::track};
  MotionDirection motion_direction{MotionDirection::zero};
  double forward_speed_command_mps{};
  bool hold_zero{};
  double effective_braking_speed_mps{};
  double stop_distance_m{};
  double profile_speed_mps{};
  double radial_error_m{};
  bool stationary{};
  double stationary_window_sec{};
  RadialStopFailure failure{RadialStopFailure::none};
  std::optional<RadialStopCertificate> certificate{};
};

class TerminalStopRegulator {
 public:
  static constexpr int certificate_version = 2;
  explicit TerminalStopRegulator(RadialStopConfig config = {})
    : config_(std::move(config)),
      detector_(config_.stationary_window_sec, config_.stationary_displacement_m,
                config_.stationary_yaw_rate_radps,
                config_.maximum_position_sample_gap_sec) {
    config_.validate();
    reset();
  }
  RadialStopState state() const { return state_; }
  void reset() {
    state_ = RadialStopState::track;
    terminal_identity_.reset();
    terminal_started_sec_.reset();
    zero_latched_sec_.reset();
    last_timestamp_sec_.reset();
    failure_ = RadialStopFailure::none;
    certificate_.reset();
    detector_.reset();
  }
  void cancel() { reset(); }
  double stopping_distance_m(double speed) const {
    require_finite(speed, "position_derived_speed_mps");
    speed = std::max(0.0, speed);
    return speed * speed / (2.0 * config_.conservative_decel_mps2) +
           config_.brake_margin_m;
  }
  double effective_braking_speed_mps(const RadialStopInput &s) const {
    if (!std::isfinite(s.position_derived_speed_mps) ||
        !std::isfinite(s.tracking_speed_command_mps) ||
        s.position_derived_speed_mps < 0.0 ||
        s.tracking_speed_command_mps < 0.0)
      throw std::invalid_argument("braking speed inputs invalid");
    return std::max(s.position_derived_speed_mps, s.tracking_speed_command_mps);
  }

  RadialStopOutput step(const RadialStopInput &s) {
    const auto previous = state_;
    validate_time(s.monotonic_time_sec); // Python raises on invalid/backward monotonic time.
    if (state_ == RadialStopState::certified || state_ == RadialStopState::hold_zero) {
      if (state_ == RadialStopState::certified) state_ = RadialStopState::hold_zero;
      return zero_output(previous, s);
    }
    if (state_ == RadialStopState::hold_fail) return zero_output(previous, s);

    if (!s.active && state_ == RadialStopState::track) {
      detector_.reset();
      return tracking_output(previous, s);
    }
    if (!s.active) return fail(previous, s, RadialStopFailure::terminal_request_lost);
    if (!valid_measurement(s)) return fail(previous, s, RadialStopFailure::invalid_measurement);
    if (!s.telemetry_fresh) return fail(previous, s, RadialStopFailure::stale_telemetry);

    if (!terminal_identity_) {
      terminal_identity_ = s.terminal_identity;
      terminal_started_sec_ = s.monotonic_time_sec;
    } else if (s.terminal_identity != terminal_identity_) {
      return fail(previous, s, RadialStopFailure::identity_changed);
    }
    if (terminal_started_sec_ &&
        s.monotonic_time_sec - *terminal_started_sec_ > config_.terminal_timeout_sec)
      return fail(previous, s, RadialStopFailure::terminal_timeout);

    const double radial = std::hypot(s.along_remaining_m, s.cross_error_m);
    const double effective = effective_braking_speed_mps(s);
    const double stop_distance = stopping_distance_m(effective);
    if (state_ == RadialStopState::zero_latch || state_ == RadialStopState::settle)
      return advance_zero_latch(previous, s, radial, stop_distance, effective);

    if (radial <= config_.radial_tolerance_m || s.along_remaining_m <= 0.0) {
      enter_zero_latch(s);
      return zero_output(previous, s, radial, stop_distance, {}, effective);
    }
    if (state_ == RadialStopState::track) {
      if (s.along_remaining_m <= config_.terminal_guidance_distance_m)
        state_ = RadialStopState::terminal_guidance;
      return tracking_output(previous, s, radial, stop_distance, effective);
    }
    if (state_ == RadialStopState::terminal_guidance) {
      if (s.along_remaining_m <= stop_distance) {
        state_ = RadialStopState::brake_profile;
        return braking_output(previous, s, radial, stop_distance, effective);
      }
      return tracking_output(previous, s, radial, stop_distance, effective);
    }
    if (state_ == RadialStopState::brake_profile)
      return braking_output(previous, s, radial, stop_distance, effective);
    throw std::logic_error("unhandled radial-stop state");
  }

 private:
  RadialStopOutput advance_zero_latch(RadialStopState previous,
      const RadialStopInput &s, double radial, double stop_distance,
      double effective) {
    if (!zero_latched_sec_) throw std::logic_error("missing zero-latch timestamp");
    if (s.monotonic_time_sec - *zero_latched_sec_ > config_.settle_timeout_sec)
      return fail(previous, s, RadialStopFailure::settle_timeout);
    PositionWindowEvidence evidence;
    try {
      evidence = detector_.update(s.position_sample_time_sec, s.position_x_m,
                                  s.position_y_m, s.measured_yaw_rate_radps);
    } catch (const std::invalid_argument &) {
      return fail(previous, s, RadialStopFailure::invalid_measurement);
    }
    if (!evidence.stationary) {
      state_ = RadialStopState::zero_latch;
      return zero_output(previous, s, radial, stop_distance, evidence, effective);
    }
    if (state_ == RadialStopState::zero_latch) {
      state_ = RadialStopState::settle;
      return zero_output(previous, s, radial, stop_distance, evidence, effective);
    }
    if (radial > config_.radial_tolerance_m)
      return fail(previous, s, RadialStopFailure::settled_outside_tolerance);
    if (!terminal_identity_) throw std::logic_error("missing terminal identity");
    certificate_ = RadialStopCertificate{certificate_version, *terminal_identity_,
      s.monotonic_time_sec, radial, s.along_remaining_m, s.cross_error_m,
      s.position_x_m, s.position_y_m, evidence.held_sec,
      evidence.maximum_displacement_m, evidence.maximum_abs_yaw_rate_radps,
      "position_derived", "px4_estimator_frame_only"};
    state_ = RadialStopState::certified;
    return zero_output(previous, s, radial, stop_distance, evidence, effective);
  }

  void enter_zero_latch(const RadialStopInput &s) {
    state_ = RadialStopState::zero_latch;
    zero_latched_sec_ = s.monotonic_time_sec;
    detector_.reset();
    detector_.update(s.position_sample_time_sec, s.position_x_m,
                     s.position_y_m, s.measured_yaw_rate_radps);
  }

  RadialStopOutput braking_output(RadialStopState previous, const RadialStopInput &s,
      double radial, double stop_distance, double effective) {
    const double remaining = std::max(0.0, s.along_remaining_m - config_.brake_margin_m);
    const double profile = std::sqrt(2.0 * config_.conservative_decel_mps2 * remaining);
    const double command = std::min(s.tracking_speed_command_mps, profile);
    if (command <= 0.0) {
      enter_zero_latch(s);
      return zero_output(previous, s, radial, stop_distance, {}, effective);
    }
    return {previous, state_, MotionDirection::forward, command, false,
            effective, stop_distance, profile, radial, false, 0.0,
            RadialStopFailure::none, {}};
  }

  RadialStopOutput tracking_output(RadialStopState previous, const RadialStopInput &s,
      std::optional<double> radial = {}, std::optional<double> stop_distance = {},
      std::optional<double> effective = {}) const {
    // Deliberately retains the Python inactive tracking-output semantics.
    const double command = std::max(0.0, s.tracking_speed_command_mps);
    return {previous, state_, command > 0.0 ? MotionDirection::forward : MotionDirection::zero,
      command, command == 0.0, (effective ? *effective : safe_effective(s)),
      (stop_distance ? *stop_distance : safe_stop_distance(s)), command,
      (radial ? *radial : safe_radial(s)), false, 0.0,
      RadialStopFailure::none, {}};
  }

  RadialStopOutput zero_output(RadialStopState previous, const RadialStopInput &s,
      std::optional<double> radial = {}, std::optional<double> stop_distance = {},
      std::optional<PositionWindowEvidence> stationary = {},
      std::optional<double> effective = {}) const {
    const PositionWindowEvidence evidence = stationary.value_or(PositionWindowEvidence{});
    return {previous, state_, MotionDirection::zero, 0.0, true,
      (effective ? *effective : safe_effective(s)), (stop_distance ? *stop_distance : safe_stop_distance(s)),
      0.0, (radial ? *radial : safe_radial(s)), evidence.stationary, evidence.held_sec,
      failure_, certificate_};
  }

  RadialStopOutput fail(RadialStopState previous, const RadialStopInput &s,
      RadialStopFailure f) {
    state_ = RadialStopState::hold_fail;
    failure_ = f;
    certificate_.reset();
    return zero_output(previous, s);
  }

  void validate_time(double now) {
    if (!std::isfinite(now) || now < 0.0)
      throw std::invalid_argument("monotonic_time_sec invalid");
    if (last_timestamp_sec_ && now < *last_timestamp_sec_)
      throw std::invalid_argument("monotonic_time_sec moved backwards");
    last_timestamp_sec_ = now;
  }
  static bool valid_measurement(const RadialStopInput &s) {
    if (!s.terminal_identity ||
        s.terminal_identity->find_first_not_of(" \t\n\r\f\v") == std::string::npos)
      return false;
    for (double v : {s.along_remaining_m, s.cross_error_m,
         s.position_sample_time_sec, s.position_x_m, s.position_y_m,
         s.position_derived_speed_mps, s.measured_yaw_rate_radps,
         s.tracking_speed_command_mps}) {
      if (!std::isfinite(v)) return false;
    }
    return s.position_derived_speed_mps >= 0.0 && s.tracking_speed_command_mps >= 0.0;
  }
  static double safe_radial(const RadialStopInput &s) {
    if (!std::isfinite(s.along_remaining_m) || !std::isfinite(s.cross_error_m))
      return std::numeric_limits<double>::infinity();
    return std::hypot(s.along_remaining_m, s.cross_error_m);
  }
  double safe_effective(const RadialStopInput &s) const {
    try { return effective_braking_speed_mps(s); }
    catch (const std::invalid_argument &) { return std::numeric_limits<double>::infinity(); }
  }
  double safe_stop_distance(const RadialStopInput &s) const {
    const double v = safe_effective(s);
    if (!std::isfinite(v)) return std::numeric_limits<double>::infinity();
    return stopping_distance_m(v);
  }

  RadialStopConfig config_{};
  PositionWindowStationaryDetector detector_;
  RadialStopState state_{RadialStopState::track};
  std::optional<std::string> terminal_identity_{};
  std::optional<double> terminal_started_sec_{}, zero_latched_sec_{}, last_timestamp_sec_{};
  RadialStopFailure failure_{RadialStopFailure::none};
  std::optional<RadialStopCertificate> certificate_{};
};

} // namespace dyx::rpp
