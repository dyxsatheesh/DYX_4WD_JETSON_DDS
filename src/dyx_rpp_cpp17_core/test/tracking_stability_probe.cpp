#include "dyx_rpp_cpp17_core/tracking_stability.hpp"

#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace dyx::rpp;

namespace {
std::string token(std::istringstream &line) {
  std::string value;
  if (!(line >> value)) throw std::invalid_argument("missing token");
  return value;
}
double number(std::istringstream &line) {
  const auto value = token(line);
  if (value == "nan") return std::numeric_limits<double>::quiet_NaN();
  if (value == "inf") return std::numeric_limits<double>::infinity();
  if (value == "-inf") return -std::numeric_limits<double>::infinity();
  std::size_t consumed = 0;
  const double output = std::stod(value, &consumed);
  if (consumed != value.size()) throw std::invalid_argument("bad numeric token");
  return output;
}
bool boolean(std::istringstream &line) {
  const auto v = token(line);
  if (v == "0") return false;
  if (v == "1") return true;
  throw std::invalid_argument("bad boolean token");
}
void emit(const TrackingControlOutput &out) {
  std::cout << tracking_state_name(out.state) << ' '
            << out.valid << ' ' << out.acceleration_allowed << ' '
            << out.recovery_speed_scale << ' ' << out.transition_reason << ' '
            << out.bounded_dt_sec << ' ' << out.stable_dwell_sec << ' '
            << out.reset_applied << ' '
            << out.invalid_reason.value_or("n") << '\n';
}
} // namespace

int main(int argc, char **argv) {
  try {
    TrackingControlConfig config{};
    if (argc > 1 && std::string(argv[1]) == "no_clamp_entry")
      config.recovery_enter_on_bearing_clamp = false;
    TrackingStabilityController controller(config);
    std::cout << std::setprecision(17);
    std::string text;
    while (std::getline(std::cin, text)) {
      if (text.empty()) continue;
      std::istringstream line(text);
      const auto action = token(line);
      if (action == "reset") {
        const auto identity = token(line);
        controller.reset(identity == "n" ? std::nullopt :
                         std::optional<std::string>(identity));
        continue;
      }
      if (action != "step") throw std::invalid_argument("unknown action");
      TrackingControlInput input{};
      input.path_identity = token(line);
      input.projection_s_m = number(line);
      input.signed_cross_track_m = number(line);
      input.heading_error_rad = number(line);
      input.commanded_speed_mps = number(line);
      input.measured_speed_mps = number(line);
      input.bearing_clamped = boolean(line);
      input.telemetry_fresh = boolean(line);
      input.dt_sec = number(line);
      input.reset = boolean(line);
      emit(controller.step(input));
    }
  } catch (const std::exception &e) {
    std::cerr << "Tracking stability probe failure: " << e.what() << '\n';
    return 2;
  }
  return 0;
}
