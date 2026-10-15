#pragma once
// Phase 2H4G: Portable JSON wire encoding, no external dependencies.
// Pure formatting only: no listener, auth, DDS, network or actuating commands.

#include "dyx_rpp_cpp17_core/backend_live_status.hpp"
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

namespace dyx::rpp::http_json {

inline bool valid_utf8(const std::string& s) {
  for (std::size_t i = 0; i < s.size();) {
    const auto c = static_cast<unsigned char>(s[i]);
    if (c < 0x80) { ++i; continue; }
    unsigned continuation = 0;
    std::uint32_t value = 0;
    if (c >= 0xC2 && c <= 0xDF) { continuation = 1; value = c & 0x1F; }
    else if (c >= 0xE0 && c <= 0xEF) { continuation = 2; value = c & 0x0F; }
    else if (c >= 0xF0 && c <= 0xF4) { continuation = 3; value = c & 0x07; }
    else return false;
    if (continuation >= s.size() - i) return false;
    for (unsigned k = 1; k <= continuation; ++k) {
      const auto b = static_cast<unsigned char>(s[i + k]);
      if ((b & 0xC0) != 0x80) return false;
      value = (value << 6) | (b & 0x3F);
    }
    if ((continuation == 1 && value < 0x80) ||
        (continuation == 2 && value < 0x800) ||
        (continuation == 3 && value < 0x10000) ||
        (value >= 0xD800 && value <= 0xDFFF) || value > 0x10FFFF)
      return false;
    i += continuation + 1;
  }
  return true;
}

inline std::string quoted(const std::string& s) {
  if (!valid_utf8(s)) throw std::invalid_argument("invalid UTF-8 response field");
  static constexpr char hex[] = "0123456789abcdef";
  std::string out;
  out.reserve(s.size() + 2);
  out.push_back('"');
  for (const unsigned char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          out += "\\u00";
          out.push_back(hex[c >> 4]);
          out.push_back(hex[c & 15]);
        } else out.push_back(static_cast<char>(c));
    }
  }
  out.push_back('"');
  return out;
}
inline const char* boolean(bool value) { return value ? "true" : "false"; }
inline std::string optional_double(bool valid, double value) {
  if (!valid || !std::isfinite(value)) return "null";
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(12) << value;
  return out.str();
}
inline std::string optional_unsigned(bool valid, unsigned value) {
  return valid ? std::to_string(value) : "null";
}
inline std::string error_json(const std::string& code) {
  return "{\"ok\":false,\"code\":" + quoted(code) + "}";
}

inline std::string mission_json(const BackendMissionApiResponse& r) {
  std::string out = "{\"ok\":true,\"code\":" + quoted(r.code) +
      ",\"mission\":{\"stage\":" + quoted(r.stage_name) +
      ",\"reason\":" + quoted(r.reason) +
      ",\"current_marking\":" + quoted(r.active_marking_label) +
      ",\"total\":" + std::to_string(r.total) +
      ",\"completed\":" + std::to_string(r.completed) +
      ",\"pending\":" + std::to_string(r.pending) +
      ",\"failed\":" + std::to_string(r.failed) +
      ",\"skipped\":" + std::to_string(r.skipped) +
      ",\"loaded\":" + boolean(r.mission_loaded) +
      ",\"recovery_quarantined\":" + boolean(r.recovery_quarantined) +
      "},\"safe_stop_intent\":" + boolean(r.safe_stop_intent) +
      ",\"decision_only\":true,\"hardware_command_emitted\":false}";
  return out;
}
inline std::string live_json(const BackendLiveStatusResponse& r) {
  const bool fresh = r.snapshot_fresh && !r.telemetry_integrity_fault;
  // Only emit current, validated sensor data. Never emit stale prior fixes.
  return "{\"ok\":true,\"code\":" + quoted(r.code) +
      ",\"mission\":{\"stage\":" + quoted(r.stage_name) +
      ",\"reason\":" + quoted(r.mission_reason) +
      ",\"current_marking\":" + quoted(r.marking_label) +
      ",\"total\":" + std::to_string(r.total) +
      ",\"completed\":" + std::to_string(r.completed) +
      ",\"pending\":" + std::to_string(r.pending) +
      ",\"failed\":" + std::to_string(r.failed) +
      ",\"skipped\":" + std::to_string(r.skipped) +
      ",\"loaded\":" + boolean(r.mission_loaded) +
      "},\"telemetry\":{\"fresh\":" + boolean(fresh) +
      ",\"integrity_fault\":" + boolean(r.telemetry_integrity_fault) +
      ",\"rtk\":" + quoted(fresh && r.gps_valid ? r.rtk_status : "UNAVAILABLE") +
      ",\"satellites_used\":" + optional_unsigned(fresh && r.satellites_valid, r.satellites_used) +
      ",\"battery_voltage_v\":" + optional_double(fresh && r.battery_valid, r.battery_voltage_v) +
      ",\"battery_remaining_percent\":" + optional_double(fresh && r.battery_remaining_valid, r.battery_remaining_percent) +
      ",\"correction_age_sec\":" + optional_double(fresh && r.corrections_valid, r.correction_age_sec) +
      ",\"vehicle_fresh\":" + boolean(fresh && r.px4_vehicle_fresh) +
      ",\"estimator_valid\":" + boolean(fresh && r.estimator_valid) +
      ",\"obstacle_clear\":" + boolean(fresh && r.obstacle_clear) +
      ",\"battery_critical\":" + boolean(fresh && r.battery_critical_latched) +
      ",\"battery_debounce_pending\":" + boolean(fresh && r.battery_debounce_pending) +
      ",\"safety_hold\":" + quoted(fresh && r.safety_hold_fresh ? r.safety_hold_reason : "NO_FRESH_SAFETY_TICK") +
      ",\"marking_accuracy_m\":" + optional_double(fresh && r.marking_accuracy_valid, r.marking_radial_error_m) +
      "},\"safe_stop_intent\":" + boolean(r.safe_stop_intent) +
      ",\"read_only\":true,\"hardware_command_emitted\":false}";
}

} // namespace dyx::rpp::http_json
