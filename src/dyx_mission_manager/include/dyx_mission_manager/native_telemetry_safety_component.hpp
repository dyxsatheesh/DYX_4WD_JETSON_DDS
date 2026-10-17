#pragma once
// Organized C++17 telemetry-safety module, separate from ROS transport.
// No HTTP, no ROS publishing, no arming, no PX4 actuator path.
// All RTCM receiver-age evidence remains disconnected until a trusted
// receiver-specific adapter is independently verified.
#include "dyx_mission_manager/native_power_gnss_observer.hpp"
#include "dyx_mission_manager/native_rtk_float_guard.hpp"
#include "dyx_mission_manager/native_rtcm_age_guard.hpp"
#include <cstdint>

namespace dyx::http_backend {
class NativeTelemetrySafetyComponent final {
 public:
  void observe_battery(double host_sec, std::uint64_t sample_us,
                       double voltage_v, bool connected, bool powering_off,
                       unsigned cell_count, unsigned warning) {
    power_.observe_battery(host_sec, sample_us, voltage_v, connected,
                           powering_off, cell_count, warning);
  }

  void observe_gps(double host_sec, std::uint64_t sample_us,
                   unsigned fix_type, bool rtcm_crc_failed, bool system_error) {
    power_.observe_gps(host_sec, sample_us, fix_type,
                       rtcm_crc_failed, system_error);
    float_guard_.observe(host_sec, sample_us, fix_type,
                         rtcm_crc_failed, system_error);
  }

  bool critical_battery_latched() const { return power_.low_voltage_latched(); }
  const NativePowerGnssObserver& power() const { return power_; }
  const NativeRtkFloatGuard& rtk() const { return float_guard_; }
  const NativeRtcmAgeGuard& rtcm() const { return receiver_age_; }

  // Deliberately no public method to supply trusted RTCM-age provenance.
  // GNSS FIX and NTRIP reception cannot independently certify this value.
  bool corrections_fresh_verified() const { return false; }

 private:
  NativePowerGnssObserver power_{};
  NativeRtkFloatGuard float_guard_{};
  NativeRtcmAgeGuard receiver_age_{};
};
}  // namespace dyx::http_backend
