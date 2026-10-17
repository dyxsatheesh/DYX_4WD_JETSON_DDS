#pragma once
#include "geo_csv.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace dyx::trajectory {
// PX4 local-position reference only. This is NOT an RTK-integrity or
// receiver-level lever-arm calibration certificate.
struct Px4OriginSample {
 double latitude_deg{999.0};
 double longitude_deg{999.0};
 std::uint64_t ref_timestamp_us{0};
 std::uint64_t sample_timestamp_us{0};
 std::uint8_t xy_reset_counter{0};
 std::uint8_t heading_reset_counter{0};
 bool xy_global{false};
 bool xy_valid{false};
 bool observed{false};
 std::chrono::steady_clock::time_point received{};
};
class Px4OriginGuard {
public:
 void observe(Px4OriginSample next) {
  if (!next.observed || !next.xy_global || !next.xy_valid ||
      next.ref_timestamp_us == 0 || next.sample_timestamp_us == 0 ||
      next.sample_timestamp_us < next.ref_timestamp_us ||
      !std::isfinite(next.latitude_deg) || !std::isfinite(next.longitude_deg) ||
      std::abs(next.latitude_deg) > 90 || std::abs(next.longitude_deg) > 180) {
   fault_ = true; return;
  }
  if (sample_.observed && (
      next.sample_timestamp_us <= sample_.sample_timestamp_us ||
      next.ref_timestamp_us != sample_.ref_timestamp_us ||
      next.xy_reset_counter != sample_.xy_reset_counter ||
      next.heading_reset_counter != sample_.heading_reset_counter ||
      next.latitude_deg != sample_.latitude_deg ||
      next.longitude_deg != sample_.longitude_deg)) {
   fault_ = true;
  }
  if (!fault_) sample_ = next;
 }
 bool good(std::chrono::steady_clock::time_point now,
           double max_age_sec = 0.5) const {
  const double age = std::chrono::duration<double>(now - sample_.received).count();
  return !fault_ && sample_.observed && age >= 0 && age <= max_age_sec;
 }
 GeoPoint origin(std::chrono::steady_clock::time_point now) const {
  if (!good(now)) throw std::runtime_error("PX4_LOCAL_ORIGIN_UNAVAILABLE_STALE_OR_RESET");
  return {sample_.latitude_deg, sample_.longitude_deg};
 }
 bool faulted() const { return fault_; }
 std::uint64_t epoch() const { return sample_.ref_timestamp_us; }
private:
 Px4OriginSample sample_{};
 bool fault_{false};
};
}
