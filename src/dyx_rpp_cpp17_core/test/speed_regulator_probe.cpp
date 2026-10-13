#include "dyx_rpp_cpp17_core/speed_regulator.hpp"
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
std::optional<double> parse_optional(const std::string &s) {
  if (s == "n") return std::nullopt;
  return std::stod(s);
}
void output_optional(const std::optional<double> &value) {
  if (value) std::cout << ' ' << *value;
  else std::cout << " n";
}
}

int main(int argc, char **argv) {
  try {
    dyx::rpp::LongitudinalRegulatorConfig cfg{};
    if (argc > 1) cfg.curvature_enabled = std::string(argv[1]) == "curvature";
    dyx::rpp::LongitudinalRegulator regulator(cfg);
    std::cout << std::setprecision(17);
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line.empty()) continue;
      std::istringstream in(line);
      std::string action;
      in >> action;
      if (action == "reset") {
        double progress{}, initial{};
        if (!(in >> progress >> initial)) throw std::runtime_error("bad reset");
        regulator.reset(progress, initial);
      } else if (action == "step") {
        // Same fifteen fields, in order, as SpeedRegulatorInput.
        std::vector<std::string> fields;
        std::string token;
        while (in >> token) fields.push_back(token);
        if (fields.size() != 15) throw std::runtime_error("step needs 15 fields");
        auto d = [&](int i) { return std::stod(fields.at(static_cast<size_t>(i))); };
        auto o = [&](int i) { return parse_optional(fields.at(static_cast<size_t>(i))); };
        const dyx::rpp::SpeedRegulatorInput request{
          d(0), d(1), d(2), d(3), d(4), d(5), d(6),
          o(7), o(8), o(9), o(10), o(11), d(12) != 0.0,
          o(13), d(14) != 0.0
        };
        const auto result = regulator.resolve(request);
        const auto &c = result.caps;
        std::cout << result.requested_speed_mps << ' '
          << result.winning_cap_owner << ' '
          << result.effective_speed_mps << ' '
          << result.bounded_dt_sec << ' '
          << result.acceleration_gate_scale << ' '
          << result.acceleration_progress_m << ' '
          << (result.recovery_active ? 1 : 0) << ' '
          << result.recovery_transition << ' '
          << result.recovery_exit_dwell_sec << ' '
          << result.recovery_exit_dwell_required_sec;
        output_optional(result.corner_required_braking_distance_m);
        output_optional(result.terminal_required_braking_distance_m);
        std::cout << ' ' << c.mission_mps << ' ' << c.hardware_mps;
        output_optional(c.recovery_mps);
        std::cout << ' ' << c.acceleration_mps << ' ' << c.heading_mps
          << ' ' << c.cross_track_mps;
        output_optional(c.tracking_mps);
        output_optional(c.corner_mps);
        output_optional(c.terminal_mps);
        output_optional(c.curvature_mps);
        std::cout << '\n';
      } else {
        throw std::runtime_error("unknown action");
      }
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "speed regulator probe: " << e.what() << '\n';
    return 2;
  }
}
