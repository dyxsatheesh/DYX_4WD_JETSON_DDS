#pragma once

// Non-actuating DYX RPP telemetry statistics; Python reference:
// rpp_controller/tracking_control.py::TrackingMetricsAccumulator.
// These statistics are DIAGNOSTIC, not mission completion authority.
#include "dyx_rpp_cpp17_core/tracking_stability.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dyx::rpp {

struct TrackingMetricsConfig {
  std::size_t metrics_quantile_window_capacity{2048};
  double metrics_histogram_bin_width_m{0.001};
  double metrics_histogram_max_m{1.0};
  double metrics_monotonic_tolerance_m{0.001};
  double metrics_cruise_speed_threshold_mps{0.80};
  double control_dt_max_sec{0.10};

  void validate() const {
    if (metrics_quantile_window_capacity == 0)
      throw std::invalid_argument("metrics_quantile_window_capacity must be positive");
    for (double x : {metrics_histogram_bin_width_m, metrics_histogram_max_m,
                     metrics_monotonic_tolerance_m, metrics_cruise_speed_threshold_mps,
                     control_dt_max_sec}) {
      if (!std::isfinite(x)) throw std::invalid_argument("nonfinite metrics configuration");
    }
    if (metrics_histogram_bin_width_m <= 0.0 || metrics_histogram_max_m <= 0.0 ||
        control_dt_max_sec <= 0.0 || metrics_monotonic_tolerance_m < 0.0 ||
        metrics_cruise_speed_threshold_mps < 0.0)
      throw std::invalid_argument("invalid metrics configuration threshold");
    const double bins = std::ceil(metrics_histogram_max_m / metrics_histogram_bin_width_m);
    if (!std::isfinite(bins) || bins > 1000000.0)
      throw std::invalid_argument("tracking metrics histogram cannot exceed 1,000,000 bins");
  }
};

struct TrackingMetricsSnapshot {
  std::optional<std::string> mission_identity{}, path_identity{};
  std::uint64_t sample_count{}, rejected_sample_count{};
  double elapsed_sec{};
  double mean_abs_cross_track_m{}, rms_cross_track_m{}, p95_abs_cross_track_m{};
  double trailing_p95_abs_cross_track_m{};
  bool p95_histogram_saturated{};
  std::uint64_t histogram_overflow_count{};
  double max_abs_cross_track_m{}, mean_abs_heading_error_rad{}, max_abs_heading_error_rad{};
  std::uint64_t monotonic_s_violation_count{};
  double recovery_time_sec{}, recapture_time_sec{}, cruise_time_sec{};
  double mean_commanded_speed_mps{}, mean_measured_speed_mps{};
  double mean_abs_speed_error_mps{}, rms_speed_error_mps{}, max_abs_speed_error_mps{};
  std::size_t quantile_sample_count{}, quantile_window_capacity{};
  std::uint64_t discontinuity_count{};
  std::optional<std::string> last_discontinuity_reason{};
  bool valid_for_acceptance{true};
};

class TrackingMetricsAccumulator {
 public:
  explicit TrackingMetricsAccumulator(TrackingMetricsConfig config = {})
      : config_(std::move(config)) {
    config_.validate();
    const auto bins = static_cast<std::size_t>(
        std::ceil(config_.metrics_histogram_max_m / config_.metrics_histogram_bin_width_m));
    histogram_.assign(bins, 0);
    reset();
  }

  void reset(std::optional<std::string> mission = {},
             std::optional<std::string> path = {}) {
    mission_identity_ = std::move(mission);
    path_identity_ = std::move(path);
    sample_count_ = rejected_ = monotonic_ = overflow_ = discontinuities_ = 0;
    elapsed_ = sum_abs_cross_ = sum_sq_cross_ = max_abs_cross_ = 0.0;
    sum_abs_heading_ = max_abs_heading_ = 0.0;
    recovery_time_ = recapture_time_ = cruise_time_ = 0.0;
    sum_cmd_ = sum_meas_ = sum_abs_speed_err_ = sum_sq_speed_err_ = max_speed_err_ = 0.0;
    max_projection_.reset();
    window_.clear();
    std::fill(histogram_.begin(), histogram_.end(), 0);
    last_discontinuity_.reset();
    valid_for_acceptance_ = true;
  }

  void note_discontinuity(const std::string &reason) {
    // Python uses reason.strip() or "unspecified"; match whitespace handling
    // for the usual ASCII diagnostic strings emitted by the rover.
    const auto first = reason.find_first_not_of(" \t\r\n\f\v");
    std::string normalized = "unspecified";
    if (first != std::string::npos) {
      const auto last = reason.find_last_not_of(" \t\r\n\f\v");
      normalized = reason.substr(first, last - first + 1);
    }
    max_projection_.reset();
    ++discontinuities_;
    last_discontinuity_ = std::move(normalized);
    valid_for_acceptance_ = false;
  }

  bool add(const TrackingControlInput &sample, TrackingControlState state,
           std::optional<std::string> mission_identity = {}) {
    const std::optional<std::string> path = sample.path_identity;
    if (mission_identity != mission_identity_ || path != path_identity_)
      reset(std::move(mission_identity), path);

    if (!sample.telemetry_fresh ||
        !std::isfinite(sample.projection_s_m) ||
        !std::isfinite(sample.signed_cross_track_m) ||
        !std::isfinite(sample.heading_error_rad) ||
        !std::isfinite(sample.commanded_speed_mps) ||
        !std::isfinite(sample.measured_speed_mps) ||
        !std::isfinite(sample.dt_sec) ||
        sample.projection_s_m < 0.0 ||
        sample.commanded_speed_mps < 0.0 ||
        sample.measured_speed_mps < 0.0 ||
        sample.dt_sec < 0.0) {
      ++rejected_;
      return false;
    }

    const double dt = std::min(sample.dt_sec, config_.control_dt_max_sec);
    const double xtrack = std::abs(sample.signed_cross_track_m);
    const double heading = std::abs(sample.heading_error_rad);
    const double speed_error = sample.commanded_speed_mps - sample.measured_speed_mps;
    const double abs_speed_error = std::abs(speed_error);

    ++sample_count_;
    elapsed_ += dt;
    sum_abs_cross_ += xtrack;
    sum_sq_cross_ += xtrack * xtrack;
    max_abs_cross_ = std::max(max_abs_cross_, xtrack);
    sum_abs_heading_ += heading;
    max_abs_heading_ = std::max(max_abs_heading_, heading);
    sum_cmd_ += sample.commanded_speed_mps;
    sum_meas_ += sample.measured_speed_mps;
    sum_abs_speed_err_ += abs_speed_error;
    sum_sq_speed_err_ += speed_error * speed_error;
    max_speed_err_ = std::max(max_speed_err_, abs_speed_error);

    if (window_.size() == config_.metrics_quantile_window_capacity)
      window_.pop_front();
    window_.push_back(xtrack);
    if (xtrack > config_.metrics_histogram_max_m) {
      ++overflow_;
    } else {
      const auto bin = std::min(
          static_cast<std::size_t>(xtrack / config_.metrics_histogram_bin_width_m),
          histogram_.size() - 1);
      ++histogram_[bin];
    }
    if (max_projection_ &&
        sample.projection_s_m < *max_projection_ - config_.metrics_monotonic_tolerance_m)
      ++monotonic_;
    max_projection_ = max_projection_ ?
        std::max(*max_projection_, sample.projection_s_m) : sample.projection_s_m;

    if (state == TrackingControlState::recovery) recovery_time_ += dt;
    else if (state == TrackingControlState::recapture_stable) recapture_time_ += dt;
    else if (sample.commanded_speed_mps >= config_.metrics_cruise_speed_threshold_mps)
      cruise_time_ += dt;
    return true;
  }

  std::size_t quantile_storage_size() const { return window_.size(); }

  TrackingMetricsSnapshot snapshot() const {
    TrackingMetricsSnapshot o;
    o.mission_identity = mission_identity_;
    o.path_identity = path_identity_;
    o.sample_count = sample_count_;
    o.rejected_sample_count = rejected_;
    o.elapsed_sec = elapsed_;
    if (sample_count_) {
      const double n = static_cast<double>(sample_count_);
      o.mean_abs_cross_track_m = sum_abs_cross_ / n;
      o.rms_cross_track_m = std::sqrt(sum_sq_cross_ / n);
      o.mean_abs_heading_error_rad = sum_abs_heading_ / n;
      o.mean_commanded_speed_mps = sum_cmd_ / n;
      o.mean_measured_speed_mps = sum_meas_ / n;
      o.mean_abs_speed_error_mps = sum_abs_speed_err_ / n;
      o.rms_speed_error_mps = std::sqrt(sum_sq_speed_err_ / n);
    }
    o.p95_abs_cross_track_m = whole_run_p95();
    o.trailing_p95_abs_cross_track_m = trailing_p95();
    o.p95_histogram_saturated = saturated();
    o.histogram_overflow_count = overflow_;
    o.max_abs_cross_track_m = max_abs_cross_;
    o.max_abs_heading_error_rad = max_abs_heading_;
    o.monotonic_s_violation_count = monotonic_;
    o.recovery_time_sec = recovery_time_;
    o.recapture_time_sec = recapture_time_;
    o.cruise_time_sec = cruise_time_;
    o.max_abs_speed_error_mps = max_speed_err_;
    o.quantile_sample_count = window_.size();
    o.quantile_window_capacity = config_.metrics_quantile_window_capacity;
    o.discontinuity_count = discontinuities_;
    o.last_discontinuity_reason = last_discontinuity_;
    o.valid_for_acceptance = valid_for_acceptance_;
    return o;
  }

 private:
  std::uint64_t rank() const {
    return std::max<std::uint64_t>(1,
        static_cast<std::uint64_t>(std::ceil(0.95 * static_cast<double>(sample_count_))));
  }
  double whole_run_p95() const {
    if (!sample_count_) return 0.0;
    std::uint64_t cumulative = 0;
    for (std::size_t i = 0; i < histogram_.size(); ++i) {
      cumulative += histogram_[i];
      if (cumulative >= rank())
        return std::min((static_cast<double>(i) + 1.0) * config_.metrics_histogram_bin_width_m,
                        config_.metrics_histogram_max_m);
    }
    return max_abs_cross_;
  }
  bool saturated() const {
    return sample_count_ > 0 && rank() > sample_count_ - overflow_;
  }
  double trailing_p95() const {
    if (window_.empty()) return 0.0;
    std::vector<double> ordered(window_.begin(), window_.end());
    std::sort(ordered.begin(), ordered.end());
    const auto nearest = std::max<std::size_t>(1,
        static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(ordered.size()))));
    return ordered[nearest - 1];
  }
  TrackingMetricsConfig config_;
  std::vector<std::uint64_t> histogram_;
  std::deque<double> window_;
  std::optional<std::string> mission_identity_{}, path_identity_{}, last_discontinuity_{};
  std::optional<double> max_projection_{};
  std::uint64_t sample_count_{}, rejected_{}, monotonic_{}, overflow_{}, discontinuities_{};
  double elapsed_{}, sum_abs_cross_{}, sum_sq_cross_{}, max_abs_cross_{};
  double sum_abs_heading_{}, max_abs_heading_{};
  double recovery_time_{}, recapture_time_{}, cruise_time_{};
  double sum_cmd_{}, sum_meas_{}, sum_abs_speed_err_{}, sum_sq_speed_err_{}, max_speed_err_{};
  bool valid_for_acceptance_{true};
};

}  // namespace dyx::rpp
