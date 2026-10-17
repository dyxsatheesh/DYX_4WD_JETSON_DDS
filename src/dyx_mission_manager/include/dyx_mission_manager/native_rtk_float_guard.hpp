#pragma once
// Diagnostic and mission-PAUSE intent only. Never grants motion permission.
#include <cmath>
#include <cstdint>
namespace dyx::http_backend {
class NativeRtkFloatGuard final {
 public:
  static constexpr double kFloatPauseSec = 5.0;
  static constexpr double kFreshSec = 1.0;
  void observe(double host_sec, std::uint64_t sample_us,
               unsigned fix_type, bool crc_failed, bool system_error) {
    if (!std::isfinite(host_sec) || host_sec < 0 || sample_us == 0 ||
        (seen_ && (sample_us <= previous_sample_us_ || host_sec < received_sec_))) {
      fault_ = true;
      return;
    }
    // A stale gap invalidates uninterrupted FLOAT duration.
    if (seen_ && host_sec - received_sec_ > kFreshSec) float_since_ = -1;
    seen_ = true;
    previous_sample_us_ = sample_us;
    received_sec_ = host_sec;
    fixed_ = fix_type == 6 && !crc_failed && !system_error;
    floating_ = fix_type == 5 && !crc_failed && !system_error;
    if (fixed_) {
      float_since_ = -1;
      float_paused_ = false;  // Clears pause intent, NOT authorization to resume.
    } else if (floating_) {
      if (float_since_ < 0) float_since_ = host_sec;
      if (host_sec - float_since_ > kFloatPauseSec) float_paused_ = true;
    } else {
      float_since_ = -1;
      // Unknown/invalid GPS never authorizes motion, and does not clear a pause latch.
    }
  }
  bool fresh(double now) const {
    return seen_ && !fault_ && std::isfinite(now) && now >= received_sec_ &&
           now - received_sec_ <= kFreshSec;
  }
  bool fixed(double now) const { return fresh(now) && fixed_; }
  bool pause_requested(double now) const {
    // Missing/stale/faulted GNSS is an immediate HOLD diagnostic.
    return !fresh(now) || (floating_ && (float_paused_ ||
      (float_since_ >= 0 && now - float_since_ > kFloatPauseSec)));
  }
  bool float_over_five_seconds(double now) const {
    return fresh(now) && floating_ && float_since_ >= 0 &&
           now - float_since_ > kFloatPauseSec;
  }
  double float_duration_sec(double now) const {
    return fresh(now) && floating_ && float_since_ >= 0
               ? now - float_since_ : 0.0;
  }
  bool fault() const { return fault_; }
  // Independent trusted RTCM-age provenance is NOT connected in this batch.
  bool corrections_fresh_verified() const { return false; }
 private:
  bool seen_{false}, fault_{false}, fixed_{false}, floating_{false}, float_paused_{false};
  std::uint64_t previous_sample_us_{0};
  double received_sec_{-1}, float_since_{-1};
};
} // namespace dyx::http_backend
