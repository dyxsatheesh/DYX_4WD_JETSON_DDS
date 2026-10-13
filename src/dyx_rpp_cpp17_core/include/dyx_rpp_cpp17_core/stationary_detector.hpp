#pragma once
// C++17 parity port of terminal_stop_regulator.py:
// PositionWindowStationaryDetector / PositionWindowEvidence.
// Records position-derived stop evidence ONLY; cannot command motors or certify
// marking success without the remaining terminal-stop state machine.

#include "dyx_rpp_cpp17_core/primitives.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace dyx::rpp {

struct PositionWindowEvidence {
  bool stationary{false};
  double held_sec{0.0};
  double maximum_displacement_m{0.0};
  double maximum_abs_yaw_rate_radps{0.0};
  std::size_t sample_count{0};
};

class PositionWindowStationaryDetector {
 public:
  PositionWindowStationaryDetector(double window_sec,
                                   double displacement_limit_m,
                                   double yaw_rate_limit_radps,
                                   double maximum_sample_gap_sec)
      : window_sec_(window_sec),
        displacement_limit_m_(displacement_limit_m),
        yaw_rate_limit_radps_(yaw_rate_limit_radps),
        maximum_sample_gap_sec_(maximum_sample_gap_sec) {
    for (const double value : {window_sec_, displacement_limit_m_,
                               yaw_rate_limit_radps_, maximum_sample_gap_sec_}) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("stationary detector limits must be finite and > 0");
      }
    }
    if (maximum_sample_gap_sec_ >= window_sec_) {
      throw std::invalid_argument("maximum_sample_gap_sec must be less than window_sec");
    }
  }

  void reset() {
    samples_.clear();
    last_timestamp_sec_.reset();
  }

  PositionWindowEvidence update(double position_sample_time_sec, double x_m,
                                double y_m, double yaw_rate_radps) {
    for (double value : {position_sample_time_sec, x_m, y_m, yaw_rate_radps}) {
      if (!std::isfinite(value)) {
        reset();
        return evidence();
      }
    }
    const double now = position_sample_time_sec;
    if (now < 0.0) {
      reset();
      return evidence();
    }
    if (last_timestamp_sec_) {
      const double gap = now - *last_timestamp_sec_;
      if (gap < 0.0) {
        throw std::invalid_argument("position_sample_time_sec must not move backwards");
      }
      if (gap == 0.0) return evidence();  // Ignore duplicate timestamp + pose.
      if (gap > maximum_sample_gap_sec_) reset();
    }
    last_timestamp_sec_ = now;
    const PositionSample sample{now, x_m, y_m, std::abs(yaw_rate_radps)};
    if (sample.abs_yaw_rate_radps > yaw_rate_limit_radps_) {
      samples_.clear();
      return evidence();
    }
    const bool moved_too_far = std::any_of(samples_.begin(), samples_.end(),
        [&](const PositionSample &prior) {
          return std::hypot(sample.x_m - prior.x_m, sample.y_m - prior.y_m) >
                 displacement_limit_m_;
        });
    if (moved_too_far) samples_.clear();
    samples_.push_back(sample);
    prune_old_samples(now);
    return evidence();
  }

  PositionWindowEvidence evidence() const {
    if (samples_.empty()) return {};
    const double held_sec = std::max(0.0, samples_.back().timestamp_sec -
                                              samples_.front().timestamp_sec);
    double max_displacement = 0.0;
    double max_yaw_rate = 0.0;
    for (std::size_t i = 0; i < samples_.size(); ++i) {
      max_yaw_rate = std::max(max_yaw_rate, samples_[i].abs_yaw_rate_radps);
      for (std::size_t j = i + 1; j < samples_.size(); ++j) {
        max_displacement = std::max(max_displacement,
            std::hypot(samples_[i].x_m - samples_[j].x_m,
                       samples_[i].y_m - samples_[j].y_m));
      }
    }
    const bool stationary = held_sec >= window_sec_ &&
                            max_displacement <= displacement_limit_m_ &&
                            max_yaw_rate <= yaw_rate_limit_radps_;
    return {stationary, held_sec, max_displacement, max_yaw_rate, samples_.size()};
  }

 private:
  struct PositionSample {
    double timestamp_sec{}, x_m{}, y_m{}, abs_yaw_rate_radps{};
  };

  void prune_old_samples(double now_sec) {
    const double cutoff = now_sec - window_sec_;
    while (samples_.size() >= 2 && samples_[1].timestamp_sec <= cutoff) {
      samples_.pop_front();
    }
  }

  double window_sec_{};
  double displacement_limit_m_{};
  double yaw_rate_limit_radps_{};
  double maximum_sample_gap_sec_{};
  std::deque<PositionSample> samples_{};
  std::optional<double> last_timestamp_sec_{};
};

}  // namespace dyx::rpp
