#pragma once
// Read-only PX4 DDS power/GNSS observer. Not a source of motion authorization.
#include <cmath>
#include <cstdint>
#include <limits>

namespace dyx::http_backend {
class NativePowerGnssObserver final {
 public:
  // 8S LiFePO4 project policy: persistent 22.0 V -> mission lockout.
  static constexpr double kCriticalV = 22.0;
  static constexpr double kDebounceSec = 30.0;
  static constexpr double kFreshSec = 1.0;

  void observe_battery(double now, std::uint64_t timestamp_us, double voltage,
                       bool connected, bool powering_off, unsigned cell_count,
                       unsigned warning) {
    if (!valid_time(now) || timestamp_us == 0 ||
        (battery_seen_ && (timestamp_us <= battery_sample_us_ || now < battery_received_))) {
      battery_fault_ = true; return;
    }
    // A missing sample breaks a continuous-low-voltage observation window.
    if (battery_seen_ && now - battery_received_ > kFreshSec) low_since_ = -1;
    battery_seen_ = true;
    battery_sample_us_ = timestamp_us;
    battery_received_ = now;
    voltage_ = voltage;
    battery_valid_ = connected && !powering_off && cell_count == 8 &&
                     std::isfinite(voltage) && voltage > 0 && voltage < 40 && warning == 0;
    if (!battery_valid_) { low_since_ = -1; return; }
    if (voltage <= kCriticalV) {
      if (low_since_ < 0) low_since_ = now;
      if (now - low_since_ >= kDebounceSec) low_latched_ = true;
    } else {
      low_since_ = -1;
    }
  }

  void observe_gps(double now, std::uint64_t timestamp_us, unsigned fix_type,
                   bool rtcm_crc_failed, bool system_error) {
    if (!valid_time(now) || timestamp_us == 0 ||
        (gps_seen_ && (timestamp_us <= gps_sample_us_ || now < gps_received_))) {
      gps_fault_ = true; return;
    }
    gps_seen_ = true;
    gps_sample_us_ = timestamp_us;
    gps_received_ = now;
    // PX4 SensorGps fix_type 6 denotes RTK fixed; fix_type 5 denotes RTK float.
    // This says nothing about RTCM stream age; that requires separate evidence.
    rtk_fixed_sample_ = fix_type == 6 && !rtcm_crc_failed && !system_error;
    gps_fix_type_ = fix_type;
  }

  bool battery_fresh(double now) const {
    return battery_seen_ && !battery_fault_ && fresh(now, battery_received_);
  }
  bool gps_fresh(double now) const {
    return gps_seen_ && !gps_fault_ && fresh(now, gps_received_);
  }
  bool battery_healthy(double now) const {
    return battery_fresh(now) && battery_valid_ && !low_latched_ && voltage_ > kCriticalV;
  }
  bool rtk_fixed(double now) const { return gps_fresh(now) && rtk_fixed_sample_; }
  bool low_voltage_latched() const { return low_latched_; }
  bool battery_fault() const { return battery_fault_; }
  bool gps_fault() const { return gps_fault_; }
  double voltage() const { return battery_seen_ ? voltage_ : std::numeric_limits<double>::quiet_NaN(); }
  unsigned fix_type() const { return gps_seen_ ? gps_fix_type_ : 0; }
  // Never infer correction freshness from GNSS FIX or RTCM CRC alone.
  bool corrections_fresh_verified() const { return false; }
 private:
  static bool valid_time(double t) { return std::isfinite(t) && t >= 0; }
  static bool fresh(double now, double received) {
    return valid_time(now) && valid_time(received) && now >= received && now - received <= kFreshSec;
  }
  bool battery_seen_{false}, gps_seen_{false};
  bool battery_valid_{false}, rtk_fixed_sample_{false};
  bool battery_fault_{false}, gps_fault_{false}, low_latched_{false};
  std::uint64_t battery_sample_us_{0}, gps_sample_us_{0};
  double battery_received_{-1}, gps_received_{-1};
  double low_since_{-1};
  double voltage_{std::numeric_limits<double>::quiet_NaN()};
  unsigned gps_fix_type_{0};
};
} // namespace dyx::http_backend
