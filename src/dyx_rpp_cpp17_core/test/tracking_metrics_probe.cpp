#include "dyx_rpp_cpp17_core/tracking_metrics.hpp"

#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace dyx::rpp;

namespace {
std::optional<std::string> decode(const std::string &s) {
  if (s == "n") return {};
  return s;
}
TrackingControlState decode_state(const std::string &s) {
  if (s == "tracking") return TrackingControlState::tracking;
  if (s == "recovery") return TrackingControlState::recovery;
  if (s == "recapture_stable") return TrackingControlState::recapture_stable;
  throw std::runtime_error("bad state");
}
void output_optional(const std::optional<std::string> &v) {
  std::cout << (v ? *v : "n");
}
void output(const TrackingMetricsSnapshot &s) {
  output_optional(s.mission_identity); std::cout << ' ';
  output_optional(s.path_identity);
  std::cout << ' ' << s.sample_count << ' ' << s.rejected_sample_count
    << ' ' << s.elapsed_sec << ' ' << s.mean_abs_cross_track_m
    << ' ' << s.rms_cross_track_m << ' ' << s.p95_abs_cross_track_m
    << ' ' << s.trailing_p95_abs_cross_track_m
    << ' ' << int(s.p95_histogram_saturated)
    << ' ' << s.histogram_overflow_count << ' ' << s.max_abs_cross_track_m
    << ' ' << s.mean_abs_heading_error_rad << ' ' << s.max_abs_heading_error_rad
    << ' ' << s.monotonic_s_violation_count << ' ' << s.recovery_time_sec
    << ' ' << s.recapture_time_sec << ' ' << s.cruise_time_sec
    << ' ' << s.mean_commanded_speed_mps << ' ' << s.mean_measured_speed_mps
    << ' ' << s.mean_abs_speed_error_mps << ' ' << s.rms_speed_error_mps
    << ' ' << s.max_abs_speed_error_mps << ' ' << s.quantile_sample_count
    << ' ' << s.quantile_window_capacity << ' ' << s.discontinuity_count << ' ';
  output_optional(s.last_discontinuity_reason);
  std::cout << ' ' << int(s.valid_for_acceptance) << '\n';
}
std::string required(std::istringstream &stream) {
  std::string s;
  if (!(stream >> s)) throw std::runtime_error("missing input token");
  return s;
}
double dbl(std::istringstream &stream) {
  return std::stod(required(stream));
}
}
int main(int argc, char **argv) {
  try {
    TrackingMetricsConfig cfg;
    if (argc > 1 && std::string(argv[1]) == "short") {
      cfg.metrics_quantile_window_capacity = 5;
      cfg.metrics_histogram_bin_width_m = 0.01;
      cfg.metrics_histogram_max_m = 0.10;
    } else if (argc > 1 && std::string(argv[1]) != "default") {
      throw std::runtime_error("unknown test configuration");
    }
    TrackingMetricsAccumulator metrics(cfg);
    std::cout << std::setprecision(17);
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line.empty()) continue;
      std::istringstream input(line);
      const std::string op = required(input);
      std::cout << (op == "add" ? "" : "n ");
      if (op == "reset") {
        const auto mission = decode(required(input));
        const auto path = decode(required(input));
        metrics.reset(mission, path);
      } else if (op == "discontinuity") {
        std::string reason;
        std::getline(input, reason);
        if (!reason.empty() && reason.front() == ' ') reason.erase(0, 1);
        metrics.note_discontinuity(reason);
      } else if (op == "add") {
        const auto mission = decode(required(input));
        TrackingControlInput sample;
        sample.path_identity = required(input);
        sample.projection_s_m = dbl(input);
        sample.signed_cross_track_m = dbl(input);
        sample.heading_error_rad = dbl(input);
        sample.commanded_speed_mps = dbl(input);
        sample.measured_speed_mps = dbl(input);
        sample.bearing_clamped = required(input) == "1";
        sample.telemetry_fresh = required(input) == "1";
        sample.dt_sec = dbl(input);
        const auto state = decode_state(required(input));
        std::cout << int(metrics.add(sample, state, mission)) << ' ';
      } else if (op != "snapshot") {
        throw std::runtime_error("unknown command: " + op);
      }
      output(metrics.snapshot());
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "metrics probe: " << e.what() << '\n';
    return 2;
  }
}
