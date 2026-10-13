#include "dyx_rpp_cpp17_core/speed_regulator.hpp"
#include <stdexcept>
#include <cmath>
#include <stdexcept>

int main() {
  using namespace dyx::rpp;
  const auto check = [](bool pass) { if (!pass) throw std::runtime_error("native regulator assertion failed"); };
  LongitudinalRegulator control;
  control.reset();
  SpeedRegulatorInput input{};
  input.mission_speed_ceiling_mps = 1.0;
  input.dt_sec = 0.05;
  input.along_track_progress_m = 0.0;
  auto out = control.resolve(input);
  check(out.requested_speed_mps > 0.0 && out.requested_speed_mps < 0.2);
  input.along_track_progress_m = 0.5;
  out = control.resolve(input);
  check(out.requested_speed_mps >= 0.0 && out.requested_speed_mps <= 1.0);
  input.hard_zero = true;
  out = control.resolve(input);
  check(out.requested_speed_mps == 0.0 && out.winning_cap_owner == "hard_zero");
  input.hard_zero = false;
  input.cross_track_error_m = 0.12;
  out = control.resolve(input);
  check(out.recovery_active && out.recovery_transition == "ENTERED");
  check(out.requested_speed_mps <= 0.15);
  input.cross_track_error_m = 0.0;
  for (int i = 0; i < 8; ++i) out = control.resolve(input);
  check(!out.recovery_active);
  input.distance_to_terminal_m = 0.0;
  out = control.resolve(input);
  check(out.caps.terminal_mps && *out.caps.terminal_mps == 0.0);
  check(out.requested_speed_mps == 0.0);
  bool invalid = false;
  try { input.mission_speed_ceiling_mps = -1.0; control.resolve(input); }
  catch (const std::invalid_argument &) { invalid = true; }
  check(invalid);
  return 0;
}
