#pragma once
// Phase 2H4F. Read-only, non-actuating Android-facing mission/telemetry data.
// This is NOT an HTTP/JSON server or an authorization implementation. Host
// timestamps, sensor provenance and principal permissions MUST be verified by
// a privileged adapter. Never use any field here as motion/tool authority.

#include "dyx_rpp_cpp17_core/backend_mission_api.hpp"
#include "dyx_rpp_cpp17_core/dds_mission_runtime.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace dyx::rpp {

constexpr const char* kBackendLiveStatusPath = "/api/v1/telemetry/live";

struct BackendLiveStatusConfig {
  double tick_fresh_sec{0.5};
  double gps_fresh_sec{1.5};
  double battery_fresh_sec{1.5};
  void validate() const {
    for (const double v : {tick_fresh_sec, gps_fresh_sec, battery_fresh_sec})
      if (!std::isfinite(v) || v <= 0.0 || v > 60.0)
        throw std::invalid_argument("invalid live-status freshness limit");
  }
};

// Supplementary PX4 SensorGps satellites_used, correlated with the SAME
// SensorGps sample as the safety telemetry; the normal safety DTO intentionally
// omits satellites. Do not substitute satellites_visible for satellites_used.
struct BackendSatellitesObservation {
  double gps_sample_received_sec{0.0};
  std::uint32_t gps_device_id{0};
  unsigned satellites_used{0};
};

// PX4 BatteryStatus.remaining [0..1], if actually available from the real
// sensor. No voltage-to-SOC conversion is performed by this UI module.
struct BackendBatteryRemainingObservation {
  double battery_sample_received_sec{0.0};
  double voltage_v{0.0};
  double remaining_fraction{std::numeric_limits<double>::quiet_NaN()};
};

struct BackendLiveStatusRequest {
  std::string method{"GET"};
  std::string path{kBackendLiveStatusPath};
  BackendVerifiedPrincipal principal{};
  double host_now_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct BackendLiveStatusResponse {
  int http_status{500};
  std::string code{"INTERNAL_ERROR"};
  bool snapshot_fresh{false};
  bool telemetry_integrity_fault{false};
  bool safe_stop_intent{false};    // NOT a physical PX4 safe stop
  bool hardware_command_emitted{false};  // always false
  bool read_only{true};

  MissionStage stage{MissionStage::unloaded};
  std::string stage_name{"unloaded"};
  std::string mission_reason{};
  std::string marking_label{};
  std::size_t total{0}, completed{0}, failed{0}, skipped{0}, pending{0};
  bool mission_loaded{false};

  bool battery_valid{false};
  double battery_voltage_v{0.0};
  bool battery_remaining_valid{false};
  double battery_remaining_percent{0.0};
  bool battery_critical_latched{false};
  bool battery_debounce_pending{false};

  bool gps_valid{false};
  std::string rtk_status{"UNAVAILABLE"};
  bool satellites_valid{false};
  unsigned satellites_used{0};
  bool corrections_valid{false};
  double correction_age_sec{0.0};
  bool px4_vehicle_fresh{false};
  bool estimator_valid{false};
  bool obstacle_clear{false};
  bool safety_hold_fresh{false};
  std::string safety_hold_reason{"NO_FRESH_SAFETY_TICK"};

  // Accuracy deliberately unavailable until a trusted position error source
  // identifies the actual marking point and certifies its own freshness.
  bool marking_accuracy_valid{false};
  double marking_radial_error_m{0.0};
};

class NativeBackendLiveStatus final {
 public:
  explicit NativeBackendLiveStatus(const NativeMissionCommandGateway& gateway,
                                   BackendLiveStatusConfig cfg = {})
      : gateway_(gateway), cfg_(cfg) { cfg_.validate(); }

  // Feed the EXACT result of gateway.tick; never synthesize or reuse safety
  // results. UI ingest never changes the mission gateway or emits commands.
  bool observe_tick(const DdsMissionRuntimeOutput& tick) {
    const double stamp = tick.telemetry.safety.now_sec;
    if (!std::isfinite(stamp) || stamp < 0.0 ||
        (last_tick_sec_ && stamp <= *last_tick_sec_)) {
      ui_integrity_fault_ = true;
      return false;
    }
    last_tick_sec_ = stamp;
    latest_tick_ = tick;
    return true;
  }

  bool observe_satellites(const BackendSatellitesObservation& f) {
    if (f.gps_device_id == 0 || f.satellites_used > 128 ||
        !std::isfinite(f.gps_sample_received_sec) ||
        f.gps_sample_received_sec < 0.0 ||
        (satellite_ && f.gps_sample_received_sec <= satellite_->gps_sample_received_sec))
      return false;
    satellite_ = f;
    return true;
  }

  bool observe_battery_remaining(const BackendBatteryRemainingObservation& f) {
    if (!std::isfinite(f.battery_sample_received_sec) ||
        f.battery_sample_received_sec < 0.0 || !std::isfinite(f.voltage_v) ||
        f.voltage_v <= 0.0 || !std::isfinite(f.remaining_fraction) ||
        f.remaining_fraction < 0.0 || f.remaining_fraction > 1.0 ||
        (battery_remaining_ &&
         f.battery_sample_received_sec <= battery_remaining_->battery_sample_received_sec))
      return false;
    battery_remaining_ = f;
    return true;
  }

  BackendLiveStatusResponse handle(const BackendLiveStatusRequest& req) const {
    BackendLiveStatusResponse out;
    if (req.path != kBackendLiveStatusPath) return error(404, "UNKNOWN_API_ROUTE");
    if (req.method != "GET") return error(405, "METHOD_NOT_ALLOWED");
    if (!req.principal.authenticated) return error(401, "AUTHENTICATION_REQUIRED");
    if (!req.principal.can_read_mission) return error(403, "READ_SCOPE_REQUIRED");
    if (!std::isfinite(req.host_now_sec) || req.host_now_sec < 0.0)
      return error(400, "INVALID_HOST_TIME");

    out.http_status = 200;
    out.safe_stop_intent = gateway_.safe_stop_requested();
    const auto mission = gateway_.mission().status();
    out.stage = mission.stage;
    out.stage_name = mission_stage_name(mission.stage);
    out.mission_reason = mission.reason;
    out.total = mission.total;
    out.completed = mission.completed;
    out.failed = mission.failed;
    out.skipped = mission.skipped;
    const auto resolved = mission.completed + mission.failed + mission.skipped;
    out.pending = resolved <= mission.total ? mission.total - resolved : 0;
    out.mission_loaded = gateway_.has_active_mission();
    const auto& records = gateway_.mission().records();
    if (out.mission_loaded && mission.current_marking < records.size())
      out.marking_label = records[mission.current_marking].goal.goal_label;

    out.telemetry_integrity_fault = ui_integrity_fault_;
    if (!last_tick_sec_ || !latest_tick_ || ui_integrity_fault_ ||
        !fresh(req.host_now_sec, *last_tick_sec_, cfg_.tick_fresh_sec) ||
        latest_tick_->telemetry.telemetry_integrity_fault) {
      out.code = "LIVE_STATUS_STALE_OR_FAULT";
      out.telemetry_integrity_fault = ui_integrity_fault_ ||
          (latest_tick_ && latest_tick_->telemetry.telemetry_integrity_fault);
      return out;
    }
    out.snapshot_fresh = true;
    out.code = "LIVE_STATUS_READ_ONLY";
    const auto& t = latest_tick_->telemetry;
    const auto& s = t.safety;
    const auto& safety = latest_tick_->decision.safety;
    out.px4_vehicle_fresh = t.px4_vehicle_fresh;
    out.estimator_valid = s.estimator_valid;
    out.obstacle_clear = std::isfinite(s.obstacle_age_sec) &&
        s.obstacle_age_sec >= 0.0 && s.obstacle_age_sec <= cfg_.tick_fresh_sec &&
        s.obstacle_clear;
    out.safety_hold_fresh = true;
    out.safety_hold_reason = safety.reason ? safety.reason : "UNKNOWN_HOLD";
    out.battery_critical_latched = safety.battery_critical_latched;
    out.battery_debounce_pending = safety.battery_debounce_pending;

    out.battery_valid = t.battery_sample_fresh &&
        fresh(req.host_now_sec, s.battery_sample_time_sec, cfg_.battery_fresh_sec) &&
        std::isfinite(s.battery_voltage_v) && s.battery_voltage_v > 0.0;
    if (out.battery_valid) {
      out.battery_voltage_v = s.battery_voltage_v;
      if (battery_remaining_ &&
          std::fabs(battery_remaining_->battery_sample_received_sec -
                    s.battery_sample_time_sec) <= 1e-6 &&
          std::fabs(battery_remaining_->voltage_v - s.battery_voltage_v) <= 0.05) {
        out.battery_remaining_valid = true;
        out.battery_remaining_percent = 100.0 * battery_remaining_->remaining_fraction;
      }
    }

    out.gps_valid = t.gps_sample_fresh &&
        fresh(req.host_now_sec, s.gnss_sample_time_sec, cfg_.gps_fresh_sec);
    if (out.gps_valid) {
      out.rtk_status = s.gnss_fix_type == 6 ? "FIXED" :
                       s.gnss_fix_type == 5 ? "FLOAT" : "OTHER";
      if (satellite_ && t.gps_source_selected &&
          satellite_->gps_device_id == t.gps_device_id &&
          std::fabs(satellite_->gps_sample_received_sec -
                    s.gnss_sample_time_sec) <= 1e-6) {
        out.satellites_valid = true;
        out.satellites_used = satellite_->satellites_used;
      }
    }
    out.corrections_valid = s.correction_stream_healthy &&
        std::isfinite(s.correction_age_sec) && s.correction_age_sec >= 0.0;
    if (out.corrections_valid) out.correction_age_sec = s.correction_age_sec;
    return out;
  }

 private:
  static BackendLiveStatusResponse error(int code, const char* reason) {
    BackendLiveStatusResponse out;
    out.http_status = code;
    out.code = reason;
    return out; // no mission or telemetry data leaked
  }
  static bool fresh(double now, double stamp, double max_age) {
    return std::isfinite(now) && std::isfinite(stamp) && stamp >= 0.0 &&
        now >= stamp && now - stamp <= max_age;
  }
  const NativeMissionCommandGateway& gateway_;
  BackendLiveStatusConfig cfg_;
  std::optional<DdsMissionRuntimeOutput> latest_tick_{};
  std::optional<double> last_tick_sec_{};
  std::optional<BackendSatellitesObservation> satellite_{};
  std::optional<BackendBatteryRemainingObservation> battery_remaining_{};
  bool ui_integrity_fault_{false};
};

} // namespace dyx::rpp
