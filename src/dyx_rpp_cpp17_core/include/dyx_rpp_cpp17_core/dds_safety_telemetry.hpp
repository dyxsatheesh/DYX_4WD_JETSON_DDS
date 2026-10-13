#pragma once
// Phase 2H4A — ROS-free, non-actuating PX4 v1.17 DDS telemetry adapter.
// Input fields correspond to px4_msgs release/1.17. The ROS 2 integration
// must project the *actual* messages, using a single host steady clock for
// receive timestamps. PX4 boot-time stamps MUST NOT be compared to host time.
// No publisher, DDS connection, servo, motor, or actuator command is present.

#include "dyx_rpp_cpp17_core/safety_arbiter.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace dyx::rpp {

struct DdsTelemetryConfig {
  double battery_fresh_sec{1.5};
  double gps_fresh_sec{1.5};
  double vehicle_fresh_sec{0.5};
  double estimator_fresh_sec{0.5};
  double corrections_fresh_sec{1.0};
  double obstacle_fresh_sec{0.5};
  unsigned expected_battery_cells{8};
  void validate() const {
    for (const double v : {battery_fresh_sec, gps_fresh_sec, vehicle_fresh_sec,
                           estimator_fresh_sec, corrections_fresh_sec,
                           obstacle_fresh_sec}) {
      if (!std::isfinite(v) || v <= 0.0 || v > 60.0)
        throw std::invalid_argument("invalid DDS telemetry freshness configuration");
    }
    if (expected_battery_cells == 0 || expected_battery_cells > 32)
      throw std::invalid_argument("invalid battery cell count");
  }
};

// Names follow PX4 release/1.17 fields; ROS adapters fill only the needed
// subset. `gps_timestamp_sample_us` maps SensorGps.timestamp_sample, NOT
// SensorGps.timestamp, so a repeatedly republished fix cannot stay fresh.
struct DdsBatteryFrame {
  std::uint64_t timestamp_us{0};
  double received_sec{0};
  double voltage_v{0};
  bool connected{false};
  bool powering_off{false};
  unsigned cell_count{0};
  unsigned warning{0};
};
struct DdsGpsFrame {
  std::uint64_t gps_timestamp_sample_us{0};
  double received_sec{0};
  std::uint32_t device_id{0};
  double latitude_deg{0}, longitude_deg{0};
  int fix_type{0};
  bool rtcm_crc_failed{false};
  std::uint32_t system_error{0};
};
struct DdsVehicleFrame {
  std::uint64_t timestamp_us{0};
  double received_sec{0};
  unsigned arming_state{0};    // VehicleStatus::ARMING_STATE_ARMED = 2
  unsigned nav_state{0};       // VehicleStatus::NAVIGATION_STATE_OFFBOARD = 14
  unsigned vehicle_type{0};    // VehicleStatus::VEHICLE_TYPE_ROVER = 3
  bool failsafe{true};
  unsigned failure_detector_status{0};
};
struct DdsEstimatorFrame {
  std::uint64_t timestamp_sample_us{0}; // VehicleLocalPosition.timestamp_sample
  double received_sec{0};
  bool xy_valid{false}, v_xy_valid{false};
  bool heading_good_for_control{false}, dead_reckoning{true};
  double x_m{0}, y_m{0}, heading_rad{0};
};
struct DdsCorrectionsFrame {
  double received_sec{0};  // host monotonic receipt time
  double correction_age_sec{std::numeric_limits<double>::infinity()};
  bool stream_healthy{false};
};
struct DdsObstacleFrame {
  double received_sec{0};
  bool clear{false};  // verified by independent obstacle adapter
};

enum class DdsTelemetryDecision {
  accepted, duplicate, invalid_data, clock_regression, px4_epoch_changed,
  fault_latched
};
struct DdsSafetySnapshot {
  SafetyInput safety{};
  bool telemetry_integrity_fault{false};
  bool gps_source_selected{false};
  std::uint32_t gps_device_id{0};
  // This is merely prepared health data, never physical/motion permission.
  bool px4_vehicle_fresh{false};
  bool gps_sample_fresh{false};
  bool battery_sample_fresh{false};
};

class NativeDdsSafetyTelemetry final {
 public:
  explicit NativeDdsSafetyTelemetry(DdsTelemetryConfig cfg = {}) : cfg_(cfg) {
    cfg_.validate();
  }
  DdsTelemetryDecision observe_battery(const DdsBatteryFrame &f) {
    const auto gate = accept_px4_stamp(f.timestamp_us, f.received_sec, battery_stamp_);
    if (gate != DdsTelemetryDecision::accepted) return gate;
    if (!std::isfinite(f.voltage_v) || f.voltage_v <= 0.0 ||
        f.voltage_v > 100.0 || f.cell_count > 32 || f.warning > 7)
      return invalidate();
    battery_ = f;
    return DdsTelemetryDecision::accepted;
  }
  DdsTelemetryDecision observe_gps(const DdsGpsFrame &f) {
    const auto gate = accept_px4_stamp(f.gps_timestamp_sample_us,
                                       f.received_sec, gps_stamp_);
    if (gate != DdsTelemetryDecision::accepted) return gate;
    if (f.device_id == 0 || f.fix_type < 0 || f.fix_type > 8 ||
        !std::isfinite(f.latitude_deg) || !std::isfinite(f.longitude_deg) ||
        std::abs(f.latitude_deg) > 90.0 || std::abs(f.longitude_deg) > 180.0)
      return invalidate();
    if (gps_device_id_ && f.device_id != *gps_device_id_)
      return invalidate(); // never silently change the master GPS source
    gps_device_id_ = f.device_id;
    gps_ = f;
    return DdsTelemetryDecision::accepted;
  }
  DdsTelemetryDecision observe_vehicle(const DdsVehicleFrame &f) {
    const auto gate = accept_px4_stamp(f.timestamp_us, f.received_sec, vehicle_stamp_);
    if (gate != DdsTelemetryDecision::accepted) return gate;
    if (f.arming_state > 2 || f.nav_state > 31 || f.vehicle_type > 3)
      return invalidate();
    vehicle_ = f;
    return DdsTelemetryDecision::accepted;
  }
  DdsTelemetryDecision observe_estimator(const DdsEstimatorFrame &f) {
    const auto gate = accept_px4_stamp(f.timestamp_sample_us,
                                       f.received_sec, estimator_stamp_);
    if (gate != DdsTelemetryDecision::accepted) return gate;
    if (!std::isfinite(f.x_m) || !std::isfinite(f.y_m) ||
        !std::isfinite(f.heading_rad) || std::abs(f.heading_rad) > 3.142)
      return invalidate();
    estimator_ = f;
    return DdsTelemetryDecision::accepted;
  }
  DdsTelemetryDecision observe_corrections(const DdsCorrectionsFrame &f) {
    const auto gate = accept_host_stamp(f.received_sec, corrections_rx_);
    if (gate != DdsTelemetryDecision::accepted) return gate;
    if (!std::isfinite(f.correction_age_sec) || f.correction_age_sec < 0.0)
      return invalidate();
    corrections_ = f;
    return DdsTelemetryDecision::accepted;
  }
  DdsTelemetryDecision observe_obstacle(const DdsObstacleFrame &f) {
    const auto gate = accept_host_stamp(f.received_sec, obstacle_rx_);
    if (gate != DdsTelemetryDecision::accepted) return gate;
    obstacle_ = f;
    return DdsTelemetryDecision::accepted;
  }

  DdsSafetySnapshot snapshot(double now_sec, bool mission_session_active,
                             bool operator_motion_enabled,
                             bool emergency_stop) const {
    DdsSafetySnapshot o;
    auto &s = o.safety;
    s.now_sec = now_sec;
    s.mission_running = mission_session_active;
    s.operator_motion_enabled = operator_motion_enabled;
    s.emergency_stop = emergency_stop;
    o.telemetry_integrity_fault = fault_latched_ || !std::isfinite(now_sec) ||
        now_sec < 0.0 || (last_host_rx_ && now_sec < *last_host_rx_);
    o.gps_source_selected = gps_device_id_.has_value();
    o.gps_device_id = gps_device_id_.value_or(0);
    if (o.telemetry_integrity_fault) return o; // All safety fields default BLOCKED.

    if (battery_) {
      o.battery_sample_fresh = fresh(now_sec, battery_->received_sec,
                                    cfg_.battery_fresh_sec);
      // PX4 battery warning critical/emergency/failed, imminent poweroff,
      // missing battery, or wrong pack topology MUST block immediately.
      // Do not wait 30 seconds for a second opinion on these separate faults.
      if (o.battery_sample_fresh && battery_->connected &&
          !battery_->powering_off && battery_->warning < 2 &&
          (battery_->cell_count == 0 ||
           battery_->cell_count == cfg_.expected_battery_cells)) {
        s.battery_voltage_v = battery_->voltage_v;
        s.battery_sample_time_sec = battery_->received_sec;
      }
    }
    if (gps_) {
      o.gps_sample_fresh = fresh(now_sec, gps_->received_sec, cfg_.gps_fresh_sec);
      if (o.gps_sample_fresh && !gps_->rtcm_crc_failed &&
          gps_->system_error == 0) {
        s.gnss_sample_time_sec = gps_->received_sec;
        s.gnss_fix_type = gps_->fix_type;
      }
    }
    if (vehicle_) {
      o.px4_vehicle_fresh = fresh(now_sec, vehicle_->received_sec,
                                 cfg_.vehicle_fresh_sec);
      if (o.px4_vehicle_fresh) {
        s.px4_age_sec = now_sec - vehicle_->received_sec;
        s.px4_offboard_armed = vehicle_->arming_state == 2 &&
            vehicle_->nav_state == 14 && vehicle_->vehicle_type == 3 &&
            !vehicle_->failsafe && vehicle_->failure_detector_status == 0;
      }
    }
    if (estimator_ && fresh(now_sec, estimator_->received_sec,
                            cfg_.estimator_fresh_sec)) {
      s.estimator_valid = estimator_->xy_valid && estimator_->v_xy_valid &&
          estimator_->heading_good_for_control && !estimator_->dead_reckoning;
    }
    if (corrections_ && fresh(now_sec, corrections_->received_sec,
                              cfg_.corrections_fresh_sec)) {
      s.correction_age_sec = corrections_->correction_age_sec +
          (now_sec - corrections_->received_sec);
      s.correction_stream_healthy = corrections_->stream_healthy;
    }
    if (obstacle_ && fresh(now_sec, obstacle_->received_sec,
                            cfg_.obstacle_fresh_sec)) {
      s.obstacle_age_sec = now_sec - obstacle_->received_sec;
      s.obstacle_clear = obstacle_->clear;
    }
    return o;
  }

  bool integrity_fault_latched() const { return fault_latched_; }
  // Only external, authenticated, PHYSICALLY DISARMED operator approval may
  // start a new PX4 boot/session epoch. It deliberately clears ALL evidence.
  bool begin_new_disarmed_px4_epoch(bool authorized, bool px4_disarmed) {
    if (!authorized || !px4_disarmed) return false;
    battery_.reset(); gps_.reset(); vehicle_.reset(); estimator_.reset();
    corrections_.reset(); obstacle_.reset();
    battery_stamp_.reset(); gps_stamp_.reset(); vehicle_stamp_.reset();
    estimator_stamp_.reset(); corrections_rx_.reset(); obstacle_rx_.reset();
    last_host_rx_.reset(); gps_device_id_.reset();
    fault_latched_ = false;
    return true;
  }
 private:
  static bool fresh(double now, double received, double max_age) {
    return std::isfinite(now) && std::isfinite(received) &&
        now >= received && received >= 0.0 && now - received <= max_age;
  }
  DdsTelemetryDecision invalidate() {
    fault_latched_ = true;
    return DdsTelemetryDecision::invalid_data;
  }
  DdsTelemetryDecision accept_clock(double received_sec) {
    if (fault_latched_) return DdsTelemetryDecision::fault_latched;
    if (!std::isfinite(received_sec) || received_sec < 0.0 ||
        (last_host_rx_ && received_sec < *last_host_rx_)) {
      fault_latched_ = true;
      return DdsTelemetryDecision::clock_regression;
    }
    last_host_rx_ = received_sec;
    return DdsTelemetryDecision::accepted;
  }
  DdsTelemetryDecision accept_px4_stamp(std::uint64_t stamp, double rx,
                                         std::optional<std::uint64_t> &last) {
    const auto clock = accept_clock(rx);
    if (clock != DdsTelemetryDecision::accepted) return clock;
    if (stamp == 0) return invalidate();
    if (last && stamp == *last) return DdsTelemetryDecision::duplicate;
    if (last && stamp < *last) {
      fault_latched_ = true;
      return DdsTelemetryDecision::px4_epoch_changed;
    }
    last = stamp;
    return DdsTelemetryDecision::accepted;
  }
  DdsTelemetryDecision accept_host_stamp(double rx, std::optional<double> &last) {
    const auto clock = accept_clock(rx);
    if (clock != DdsTelemetryDecision::accepted) return clock;
    if (last && rx == *last) return DdsTelemetryDecision::duplicate;
    last = rx;
    return DdsTelemetryDecision::accepted;
  }
  DdsTelemetryConfig cfg_{};
  std::optional<DdsBatteryFrame> battery_{};
  std::optional<DdsGpsFrame> gps_{};
  std::optional<DdsVehicleFrame> vehicle_{};
  std::optional<DdsEstimatorFrame> estimator_{};
  std::optional<DdsCorrectionsFrame> corrections_{};
  std::optional<DdsObstacleFrame> obstacle_{};
  std::optional<std::uint64_t> battery_stamp_{}, gps_stamp_{}, vehicle_stamp_{},
      estimator_stamp_{};
  std::optional<double> corrections_rx_{}, obstacle_rx_{}, last_host_rx_{};
  std::optional<std::uint32_t> gps_device_id_{};
  bool fault_latched_{false};
};
} // namespace dyx::rpp
