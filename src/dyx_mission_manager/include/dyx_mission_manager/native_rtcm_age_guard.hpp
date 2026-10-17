#pragma once
// Batch 8C: receiver-age diagnostic guard only; not a motion permission source.
// Never derive correction age from RTCM byte reception or RTK FIX status.
#include <cmath>
#include <cstdint>
#include <limits>

namespace dyx::http_backend {
class NativeRtcmAgeGuard final {
 public:
  static constexpr double kMaxRtkCorrectionAgeSec = 20.0;
  static constexpr double kMaxReportAgeSec = 1.0;

  // The caller must prove this comes from a receiver-reported age field,
  // not an NTRIP transport timestamp or user-controlled HTTP/ROS topic.
  void observe_receiver_report(double host_sec, std::uint64_t sample_us,
                               double receiver_correction_age_sec,
                               bool source_verified) {
    if (!source_verified) { source_fault_ = true; return; }
    if (!std::isfinite(host_sec) || host_sec < 0.0 || sample_us == 0 ||
        (seen_ && (sample_us <= last_sample_us_ || host_sec < last_host_sec_))) {
      source_fault_ = true;
      return;
    }
    seen_ = true;
    last_sample_us_ = sample_us;
    last_host_sec_ = host_sec;
    age_ = receiver_correction_age_sec;
    age_valid_ = std::isfinite(age_) && age_ >= 0.0 && age_ <= 600.0;
  }
  bool report_fresh(double now) const {
    return seen_ && !source_fault_ && std::isfinite(now) &&
           now >= last_host_sec_ && now - last_host_sec_ <= kMaxReportAgeSec;
  }
  // Account for elapsed monotonic time since receiver reported the age.
  double effective_age_sec(double now) const {
    if (!report_fresh(now) || !age_valid_)
      return std::numeric_limits<double>::quiet_NaN();
    return age_ + (now - last_host_sec_);
  }
  bool receiver_corrections_fresh(double now) const {
    return report_fresh(now) && age_valid_ &&
           effective_age_sec(now) <= kMaxRtkCorrectionAgeSec;
  }
  bool source_fault() const { return source_fault_; }
  bool receiver_report_connected() const { return seen_; }
 private:
  bool seen_{false}, source_fault_{false}, age_valid_{false};
  std::uint64_t last_sample_us_{0};
  double last_host_sec_{-1.0};
  double age_{std::numeric_limits<double>::quiet_NaN()};
};
} // namespace dyx::http_backend
