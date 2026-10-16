#include "dyx_rpp_cpp17_core/native_rpp_executor.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace dyx::rpp;

static NativeRppExecutionInput valid(
    double t, std::uint64_t seq) {

  NativeRppExecutionInput in{};
  in.sequence = seq;

  in.guidance.now_sec = t;
  in.guidance.dt_sec = 0.02;
  in.guidance.rover_yaw_enu_rad = 0.0;
  in.guidance.target_yaw_enu_rad = 0.05;
  in.guidance.measured_yaw_rate_enu_radps = 0.0;
  in.guidance.requested_forward_speed_mps = 0.6;
  in.guidance.motion_enabled = true;
  in.guidance.telemetry_fresh = true;
  in.guidance.stop_requested = false;
  in.guidance.pivot_permission = false;

  in.safety = {
      true, true, true, true, true,
      true, true, true, true
  };

  return in;
}

int main() {
  NativeRppExecutor rover;

  // Startup must not produce motion.
  assert(!rover.step(valid(1.0, 1)).permitted);

  // Disarm/authorization evidence is required.
  assert(!rover.begin_new_disarmed_run(false, true));
  assert(!rover.begin_new_disarmed_run(true, false));
  assert(rover.begin_new_disarmed_run(true, true));

  auto out = rover.step(valid(2.0, 1));

  assert(out.permitted);
  assert(out.north_mps >= 0.0);
  assert(std::abs(out.east_mps - 0.6) < 1e-8);
  assert(std::abs(out.yaw_rate_enu_radps) <= 0.15);

  // Normal consecutive 50 Hz tick.
  out = rover.step(valid(2.02, 2));
  assert(out.permitted);

  // RTK FLOAT immediately inhibits and latches.
  auto bad = valid(2.04, 3);
  bad.safety.rtk_fixed = false;
  assert(!rover.step(bad).permitted);
  assert(!rover.active());

  // FIXED cannot automatically resume.
  assert(!rover.step(valid(2.06, 4)).permitted);

  // Explicit disarmed recovery.
  assert(rover.begin_new_disarmed_run(true, true));
  assert(rover.step(valid(3.0, 1)).permitted);

  // STOP inhibits the command.
  auto stop = valid(3.02, 2);
  stop.guidance.stop_requested = true;
  assert(!rover.step(stop).permitted);
  assert(!rover.active());

  // Rejected stale telemetry.
  assert(rover.begin_new_disarmed_run(true, true));
  auto stale = valid(4.0, 1);
  stale.guidance.telemetry_fresh = false;
  assert(!rover.step(stale).permitted);

  // Reverse demanded by planner input must fail closed.
  assert(rover.begin_new_disarmed_run(true, true));
  auto reverse = valid(5.0, 1);
  reverse.guidance.requested_forward_speed_mps = -0.2;
  assert(!rover.step(reverse).permitted);
  assert(!rover.active());

  // Large heading changes cannot bypass pivot permission.
  assert(rover.begin_new_disarmed_run(true, true));
  auto pivot = valid(6.0, 1);
  pivot.guidance.target_yaw_enu_rad = 0.7;
  auto pivot_result = rover.step(pivot);
  assert(pivot_result.permitted);
  assert(std::abs(pivot_result.east_mps) < 1e-9);
  assert(std::abs(pivot_result.north_mps) < 1e-9);
  assert(std::abs(pivot_result.yaw_rate_enu_radps) < 1e-9);

  std::cout
      << "PASS: Native RPP executor integration tests\n";
}
