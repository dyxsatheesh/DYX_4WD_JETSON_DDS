#pragma once
// Phase 2G1B: ROS-free / Python-free, NON-ACTUATING raw RTK FIXED GNSS gate.
// Reference: mission_manager/survey_truth.py, raw_gnss_spray_gate.py.
// Survey truth is computed from post-stop raw GNSS only. EKF/RPP 20 mm
// estimator-frame certificate NEVER substitutes for raw survey evidence.
// No GNSS driver, DDS publisher, spray writer, mission advance or motor action.

#include "dyx_rpp_cpp17_core/primitives.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dyx::rpp {

struct SurveyTarget {
  std::string point_id;
  int point_index{-1};
  double latitude_deg{};
  double longitude_deg{};
};

struct RawGnssFix {
  double monotonic_sec{};
  double latitude_deg{};
  double longitude_deg{};
  int fix_type{};                  // PX4 GPS fix_type: 6 = RTK FIXED.
  int satellites{};
  double horizontal_accuracy_m{};
};

struct RawSurveyConfig {
  double tolerance_m{0.030};
  double window_sec{0.50};
  std::size_t minimum_samples{3};
  double max_scatter_m{0.020};
  double timeout_sec{3.0};
  int required_fix_type{6};

  void validate() const {
    for (double v : {tolerance_m, window_sec, max_scatter_m, timeout_sec}) {
      if (!std::isfinite(v) || v <= 0.0)
        throw std::invalid_argument("raw survey configuration must be finite positive");
    }
    // Commercial marking contract: never loosen the 30 mm raw-survey limit
    // or let a single GNSS fix certify a marking point.
    if (tolerance_m > 0.030 + 1.0e-12 || minimum_samples < 3 ||
        timeout_sec < window_sec || required_fix_type != 6)
      throw std::invalid_argument("invalid RTK FIXED survey configuration");
  }
};

enum class RawSurveyVerdict { waiting, pass, fail };
inline const char *raw_survey_verdict_name(RawSurveyVerdict s) {
  switch (s) {
    case RawSurveyVerdict::waiting: return "WAITING";
    case RawSurveyVerdict::pass: return "PASS";
    case RawSurveyVerdict::fail: return "FAIL";
  }
  return "FAIL";
}

struct RawSurveyDecision {
  RawSurveyVerdict verdict{RawSurveyVerdict::waiting};
  std::string terminal_identity{};
  std::string reason{"NOT_EVALUATED"};
  double elapsed_sec{};
  double tolerance_m{0.030};
  bool validated_raw_fixed{false};
  std::optional<double> radial_error_m{};
  std::optional<double> last_sample_monotonic_sec{};
  std::optional<double> stopped_latitude_deg{}, stopped_longitude_deg{};
  std::optional<double> north_error_m{}, east_error_m{};
  std::optional<double> along_track_error_m{}, cross_track_error_m{};
  std::optional<double> approach_bearing_ned_rad{};
  std::string bearing_source{};
  double sample_span_sec{};
  double scatter_m{};
  std::size_t sample_count{}, trimmed_count{};
  int newest_fix_type{}, newest_satellites{};
  double newest_horizontal_accuracy_m{};

  bool final() const { return verdict != RawSurveyVerdict::waiting; }
  bool passed() const { return verdict == RawSurveyVerdict::pass && validated_raw_fixed; }
};

namespace survey_detail {
constexpr double a = 6378137.0;
constexpr double flattening = 1.0 / 298.257223563;
constexpr double b = (1.0 - flattening) * a;
constexpr double e2 = flattening * (2.0 - flattening);
inline bool valid_geo(double lat, double lon) {
  return std::isfinite(lat) && std::isfinite(lon) &&
         lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0;
}
inline std::pair<double, double> local_offset_ne(double from_lat, double from_lon,
                                                  double to_lat, double to_lon) {
  const double lat_rad = ((from_lat + to_lat) * 0.5) * pi / 180.0;
  const double sin_lat = std::sin(lat_rad);
  const double w2 = 1.0 - e2 * sin_lat * sin_lat;
  const double w = std::sqrt(w2);
  const double north_per_deg = a * (1.0 - e2) / (w2 * w) * pi / 180.0;
  const double east_per_deg = a / w * std::cos(lat_rad) * pi / 180.0;
  return {(to_lat - from_lat) * north_per_deg,
          (to_lon - from_lon) * east_per_deg};
}
// Vincenty inverse geodesic, matching the Python DYX DIRECT WGS84 RAW OVERALL.
// Non-convergence rejects measurement; it must not fall back to EKF position.
inline double wgs84_inverse_distance_m(double lat1, double lon1,
                                       double lat2, double lon2) {
  if (!valid_geo(lat1, lon1) || !valid_geo(lat2, lon2))
    throw std::invalid_argument("invalid geodetic coordinates");
  if (lat1 == lat2 && lon1 == lon2) return 0.0;
  const double phi1 = lat1 * pi / 180.0;
  const double phi2 = lat2 * pi / 180.0;
  const double delta_deg = (lon2 - lon1) -
      360.0 * std::floor(((lon2 - lon1) + 180.0) / 360.0);
  const double L = delta_deg * pi / 180.0;
  const double U1 = std::atan((1.0 - flattening) * std::tan(phi1));
  const double U2 = std::atan((1.0 - flattening) * std::tan(phi2));
  const double su1 = std::sin(U1), cu1 = std::cos(U1);
  const double su2 = std::sin(U2), cu2 = std::cos(U2);
  double lambda = L;
  double sin_sigma = 0.0, cos_sigma = 0.0, sigma = 0.0;
  double cos_sq_alpha = 0.0, cos_2sigma_m = 0.0;
  bool converged = false;
  for (int i = 0; i < 100; ++i) {
    const double sl = std::sin(lambda), cl = std::cos(lambda);
    sin_sigma = std::hypot(cu2 * sl, cu1 * su2 - su1 * cu2 * cl);
    if (sin_sigma <= 1.0e-20) return 0.0;
    cos_sigma = su1 * su2 + cu1 * cu2 * cl;
    sigma = std::atan2(sin_sigma, cos_sigma);
    const double sin_alpha = cu1 * cu2 * sl / sin_sigma;
    cos_sq_alpha = std::max(0.0, 1.0 - sin_alpha * sin_alpha);
    cos_2sigma_m = cos_sq_alpha <= 1.0e-20 ? 0.0 :
        cos_sigma - 2.0 * su1 * su2 / cos_sq_alpha;
    const double C = flattening / 16.0 * cos_sq_alpha *
        (4.0 + flattening * (4.0 - 3.0 * cos_sq_alpha));
    const double previous = lambda;
    lambda = L + (1.0 - C) * flattening * sin_alpha *
        (sigma + C * sin_sigma *
         (cos_2sigma_m + C * cos_sigma * (-1.0 + 2.0 * cos_2sigma_m * cos_2sigma_m)));
    if (std::abs(lambda - previous) <= 1.0e-12) {
      converged = true;
      break;
    }
  }
  if (!converged) throw std::runtime_error("Vincenty inverse did not converge");
  const double u2 = cos_sq_alpha * (a * a - b * b) / (b * b);
  const double A = 1.0 + u2 / 16384.0 *
      (4096.0 + u2 * (-768.0 + u2 * (320.0 - 175.0 * u2)));
  const double B = u2 / 1024.0 *
      (256.0 + u2 * (-128.0 + u2 * (74.0 - 47.0 * u2)));
  const double delta_sigma = B * sin_sigma *
      (cos_2sigma_m + B / 4.0 *
       (cos_sigma * (-1.0 + 2.0 * cos_2sigma_m * cos_2sigma_m) -
        B / 6.0 * cos_2sigma_m * (-3.0 + 4.0 * sin_sigma * sin_sigma) *
        (-3.0 + 4.0 * cos_2sigma_m * cos_2sigma_m)));
  const double d = b * A * (sigma - delta_sigma);
  if (!std::isfinite(d)) throw std::runtime_error("nonfinite inverse distance");
  return std::max(0.0, d);
}
inline bool valid_fix(const RawGnssFix &s) {
  return std::isfinite(s.monotonic_sec) &&
         valid_geo(s.latitude_deg, s.longitude_deg) &&
         std::isfinite(s.horizontal_accuracy_m) && s.horizontal_accuracy_m >= 0.0 &&
         s.satellites > 0;
}
inline RawSurveyDecision unavailable(const std::string &identity,
                                      const std::string &why,
                                      double elapsed, const RawSurveyConfig &cfg) {
  RawSurveyDecision d;
  d.terminal_identity = identity;
  d.elapsed_sec = elapsed;
  d.tolerance_m = cfg.tolerance_m;
  d.reason = why;
  if (elapsed >= cfg.timeout_sec) {
    d.verdict = RawSurveyVerdict::fail;
    d.reason = "RAW_GNSS_UNAVAILABLE:" + why;
  }
  return d;
}
} // namespace survey_detail

// Every evaluation is restricted to raw GNSS samples at/after the certified stop.
// Stationary-window trimming of even one OLD sample invalidates the whole gate:
// the Python spray gate forbids hiding the last centimetres of braking.
inline RawSurveyDecision evaluate_raw_survey(
    const std::string &terminal_identity, double stop_monotonic_sec,
    double now_monotonic_sec, const std::optional<SurveyTarget> &target,
    const std::optional<SurveyTarget> &previous_target,
    const std::vector<RawGnssFix> &raw_fixes,
    const RawSurveyConfig &cfg = {},
    std::optional<double> fallback_bearing_ned_rad = {},
    std::optional<std::string> unavailable_reason = {}) {
  cfg.validate();
  if (terminal_identity.empty() || !std::isfinite(stop_monotonic_sec) ||
      !std::isfinite(now_monotonic_sec) || stop_monotonic_sec < 0.0 ||
      now_monotonic_sec < stop_monotonic_sec) {
    RawSurveyDecision d;
    d.verdict = RawSurveyVerdict::fail;
    d.terminal_identity = terminal_identity;
    d.reason = "INVALID_STOP_IDENTITY_OR_TIME";
    return d;
  }
  const double elapsed = now_monotonic_sec - stop_monotonic_sec;
  auto unavailable = [&](const std::string &reason) {
    return survey_detail::unavailable(terminal_identity, reason, elapsed, cfg);
  };
  // Python gate collects for half a second before any survey evaluation.
  if (elapsed < cfg.window_sec) return unavailable("COLLECTING_STATIONARY_RAW_GNSS");
  if (unavailable_reason && !unavailable_reason->empty()) return unavailable(*unavailable_reason);
  if (!target) return unavailable("NO_SURVEY_TARGET");
  if (target->point_id.empty() || target->point_index < 0 ||
      !survey_detail::valid_geo(target->latitude_deg, target->longitude_deg))
    return unavailable("INVALID_SURVEY_TARGET");

  std::vector<RawGnssFix> window;
  for (const auto &fix : raw_fixes) {
    if (!std::isfinite(fix.monotonic_sec) ||
        fix.monotonic_sec < stop_monotonic_sec ||
        fix.monotonic_sec > now_monotonic_sec ||
        now_monotonic_sec - fix.monotonic_sec > cfg.window_sec)
      continue;
    // A non-FIXED or otherwise invalid post-stop sample must not disappear
    // during filtering, hiding a degraded interval from the decision.
    if (!survey_detail::valid_fix(fix)) return unavailable("INVALID_RAW_GNSS_SAMPLE");
    window.push_back(fix);
  }
  if (window.size() < cfg.minimum_samples) return unavailable("INSUFFICIENT_GNSS_SAMPLES");
  if (std::any_of(window.begin(), window.end(), [&](const RawGnssFix &f) {
        return f.fix_type != cfg.required_fix_type;
      })) return unavailable("GNSS_NOT_RTK_FIXED");
  std::stable_sort(window.begin(), window.end(),
                   [](const RawGnssFix &a, const RawGnssFix &b) {
                     return a.monotonic_sec < b.monotonic_sec;
                   });
  // Samples must be individually timestamped sensor fixes, not repeated
  // copies of a cached GNSS message masquerading as independent evidence.
  for (std::size_t i = 1; i < window.size(); ++i) {
    if (window[i].monotonic_sec - window[i - 1].monotonic_sec <= 1.0e-6)
      return unavailable("GNSS_DUPLICATE_SAMPLE_TIMESTAMPS");
  }
  auto centroid = [](const std::vector<RawGnssFix> &v) {
    double lat = 0.0, lon = 0.0;
    for (const auto &fix : v) {
      lat += fix.latitude_deg;
      lon += fix.longitude_deg;
    }
    lat /= static_cast<double>(v.size());
    lon /= static_cast<double>(v.size());
    double scatter = 0.0;
    for (const auto &fix : v) {
      const auto ne = survey_detail::local_offset_ne(lat, lon, fix.latitude_deg, fix.longitude_deg);
      scatter = std::max(scatter, std::hypot(ne.first, ne.second));
    }
    return std::vector<double>{lat, lon, scatter};
  };
  auto c = centroid(window);
  std::size_t trimmed = 0;
  while (c[2] > cfg.max_scatter_m && window.size() > cfg.minimum_samples) {
    window.erase(window.begin());
    ++trimmed;
    c = centroid(window);
  }
  // Even if a nominal stationary subset exists, trimmed braking samples
  // invalidate RAW GNSS spray authorization, mirroring raw_gnss_spray_gate.py.
  if (c[2] > cfg.max_scatter_m || trimmed != 0)
    return unavailable("GNSS_WINDOW_NOT_STATIONARY");

  double radial = 0.0;
  try {
    radial = survey_detail::wgs84_inverse_distance_m(
        target->latitude_deg, target->longitude_deg, c[0], c[1]);
  } catch (const std::exception &) {
    return unavailable("GEODESIC_NOT_AVAILABLE");
  }
  if (!std::isfinite(radial)) return unavailable("NONFINITE_SURVEY_DISTANCE");

  RawSurveyDecision d;
  d.terminal_identity = terminal_identity;
  d.elapsed_sec = elapsed;
  d.tolerance_m = cfg.tolerance_m;
  d.radial_error_m = radial;
  d.stopped_latitude_deg = c[0];
  d.stopped_longitude_deg = c[1];
  const auto ne = survey_detail::local_offset_ne(
      target->latitude_deg, target->longitude_deg, c[0], c[1]);
  d.north_error_m = ne.first;
  d.east_error_m = ne.second;
  std::optional<double> bearing;
  if (previous_target &&
      survey_detail::valid_geo(previous_target->latitude_deg, previous_target->longitude_deg)) {
    const auto segment = survey_detail::local_offset_ne(previous_target->latitude_deg,
        previous_target->longitude_deg, target->latitude_deg, target->longitude_deg);
    if (std::hypot(segment.first, segment.second) >= 0.10) {
      bearing = std::atan2(segment.second, segment.first);
      d.bearing_source = "SURVEYED_SEGMENT";
    }
  }
  if (!bearing && fallback_bearing_ned_rad && std::isfinite(*fallback_bearing_ned_rad)) {
    bearing = *fallback_bearing_ned_rad;
    d.bearing_source = "ROVER_HEADING";
  }
  if (bearing) {
    d.approach_bearing_ned_rad = *bearing;
    d.along_track_error_m = -(ne.first * std::cos(*bearing) +
                               ne.second * std::sin(*bearing));
    d.cross_track_error_m = -ne.first * std::sin(*bearing) +
                              ne.second * std::cos(*bearing);
  }
  d.sample_count = window.size();
  d.trimmed_count = trimmed;
  d.sample_span_sec = window.back().monotonic_sec - window.front().monotonic_sec;
  d.scatter_m = c[2];
  d.newest_fix_type = window.back().fix_type;
  d.newest_satellites = window.back().satellites;
  d.newest_horizontal_accuracy_m = window.back().horizontal_accuracy_m;
  d.last_sample_monotonic_sec = window.back().monotonic_sec;
  d.validated_raw_fixed = true;
  if (radial <= cfg.tolerance_m) {
    d.verdict = RawSurveyVerdict::pass;
    d.reason = "PASS";
  } else {
    d.verdict = RawSurveyVerdict::fail;
    d.reason = "RAW_GNSS_RADIAL_EXCEEDS_TOLERANCE";
  }
  return d;
}

// Optional, one-shot wrapper: first decisive PASS/FAIL is latched, even if
// later samples would reverse the verdict. No automatic reset/resample.
class OneShotRawSurveyGate {
 public:
  explicit OneShotRawSurveyGate(RawSurveyConfig cfg = {}) : cfg_(cfg) { cfg_.validate(); }
  void start(std::string terminal_identity, double stop_monotonic_sec) {
    if (terminal_identity.empty() || !std::isfinite(stop_monotonic_sec) ||
        stop_monotonic_sec < 0.0)
      throw std::invalid_argument("invalid raw GNSS gate start");
    identity_ = std::move(terminal_identity);
    stop_monotonic_sec_ = stop_monotonic_sec;
    latched_.reset();
    last_now_.reset();
  }
  RawSurveyDecision step(double now, const std::optional<SurveyTarget> &target,
                         const std::optional<SurveyTarget> &previous,
                         const std::vector<RawGnssFix> &fixes,
                         std::optional<double> fallback_bearing_ned_rad = {},
                         std::optional<std::string> unavailable_reason = {}) {
    if (!identity_ || !stop_monotonic_sec_ ||
        !std::isfinite(now) || now < *stop_monotonic_sec_ ||
        (last_now_ && now < *last_now_)) {
      RawSurveyDecision failure;
      failure.verdict = RawSurveyVerdict::fail;
      failure.terminal_identity = identity_.value_or("");
      failure.reason = "INVALID_OR_BACKWARD_GATE_TIME";
      latched_ = failure;
      return failure;
    }
    if (latched_) return *latched_;
    last_now_ = now;
    auto d = evaluate_raw_survey(*identity_, *stop_monotonic_sec_, now,
                                  target, previous, fixes, cfg_,
                                  fallback_bearing_ned_rad, unavailable_reason);
    if (d.final()) latched_ = d;
    return d;
  }
 private:
  RawSurveyConfig cfg_{};
  std::optional<std::string> identity_{};
  std::optional<double> stop_monotonic_sec_{};
  std::optional<double> last_now_{};
  std::optional<RawSurveyDecision> latched_{};
};

} // namespace dyx::rpp
