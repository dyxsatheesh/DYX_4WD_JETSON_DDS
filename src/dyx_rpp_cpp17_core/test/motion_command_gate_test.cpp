#include "dyx_rpp_cpp17_core/motion_command_gate.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace dyx::rpp;

static MotionDemand valid(double time, std::uint64_t seq) {
  MotionDemand d{};
  d.host_time_sec = time;
  d.sequence = seq;
  d.forward_mps = 0.6;
  d.yaw_rate_enu_radps = 0.10;
  d.current_yaw_enu_rad = 0.0;
  d.safety = {true, true, true, true, true,
              true, true, true, true};
  return d;
}

int main() {
  MotionCommandGate g;

  auto a = valid(1.0, 1);

  assert(!g.evaluate(a).permitted);
  assert(g.latched());

  assert(!g.reset_for_new_run(false, true));
  assert(!g.reset_for_new_run(true, false));
  assert(g.reset_for_new_run(true, true));

  auto result = g.evaluate(a);
  assert(result.permitted);
  assert(std::abs(result.velocity_north_mps) < 1e-9);
  assert(std::abs(result.velocity_east_mps - 0.6) < 1e-9);

  assert(!g.evaluate(a).permitted);
  assert(g.latched());

  assert(g.reset_for_new_run(true, true));
  a = valid(2.0, 1);
  a.forward_mps = -0.1;
  assert(!g.evaluate(a).permitted);

  assert(g.reset_for_new_run(true, true));
  a = valid(3.0, 1);
  a.forward_mps = 1.01;
  assert(!g.evaluate(a).permitted);

  assert(g.reset_for_new_run(true, true));
  a = valid(4.0, 1);
  a.yaw_rate_enu_radps = 0.19;
  assert(!g.evaluate(a).permitted);

  assert(g.reset_for_new_run(true, true));
  a = valid(5.0, 1);
  a.safety.rtk_fixed = false;
  assert(!g.evaluate(a).permitted);

  assert(g.reset_for_new_run(true, true));
  a = valid(6.0, 1);
  a.safety.physical_estop_released = false;
  assert(!g.evaluate(a).permitted);

  assert(g.reset_for_new_run(true, true));
  a = valid(7.0, 1);
  assert(g.evaluate(a).permitted);

  a = valid(7.2, 2);
  assert(!g.evaluate(a).permitted);

  assert(g.reset_for_new_run(true, true));
  a = valid(8.0, 1);
  a.forward_mps = std::numeric_limits<double>::quiet_NaN();
  assert(!g.evaluate(a).permitted);

  assert(g.reset_for_new_run(true, true));
  a = valid(9.0, 1);
  a.safety.marking_reference_calibrated = false;
  assert(!g.evaluate(a).permitted);

  std::cout << "PASS: Native C++17 motion command safety tests\n";
}
