#include "dyx_rpp_cpp17_core/native_control_owner.hpp"

#include <cassert>
#include <iostream>

using namespace dyx::rpp;

static NativeMissionRppInput guidance(double now) {
  NativeMissionRppInput request{};
  request.sequence = 1;
  request.guidance.now_sec = now;
  request.guidance.dt_sec = 0.02;
  request.guidance.rover_yaw_enu_rad = 0.0;
  request.guidance.target_yaw_enu_rad = 0.0;
  request.guidance.requested_forward_speed_mps = 0.6;
  request.guidance.telemetry_fresh = true;
  request.guidance.motion_enabled = true;
  request.trusted_operator_lease = true;
  request.trusted_estop_released = true;
  request.calibrated_marking_reference = true;
  return request;
}

static MissionSequenceInput evidence(double now) {
  MissionSequenceInput e{};
  e.monotonic_time_sec = now;
  e.emergency_stop = false;
  e.measured_forward_speed_mps = 0.0;
  return e;
}

int main() {
  NativeControlOwner owner;

  // No telemetry and no mission: motion must remain blocked.
  auto first = owner.tick(
      1.0, false, false, true,
      evidence(1.0), guidance(1.0));

  assert(!first.motion.permitted);
  assert(!owner.hardware_command_emitted());

  // A prevalidated-looking mission must still require the
  // specified trusted load attestations.
  const std::vector<MarkingGoal> goals{
      {0, 0, "P0001", "RUN:P0001"},
      {10, 1, "P0002", "RUN:P0002"}};

  assert(owner.load(
      "run_owner_001", "session_owner_001",
      goals, MissionMode::automatic,
      false, true, true) ==
      ClientCommandStatus::unauthorized);

  assert(owner.load(
      "run_owner_001", "session_owner_001",
      goals, MissionMode::automatic,
      true, true, true) ==
      ClientCommandStatus::accepted);

  assert(owner.has_active_mission());

  // Even an accepted mission upload is not START authorization.
  auto loaded = owner.tick(
      2.0, true, true, false,
      evidence(2.0), guidance(2.0));

  assert(!loaded.motion.permitted);

  // STOP always removes mission authority.
  owner.force_stop();

  assert(!owner.has_active_mission());
  assert(owner.safe_stop_requested());

  auto stopped = owner.tick(
      3.0, true, true, false,
      evidence(3.0), guidance(3.0));

  assert(!stopped.motion.permitted);

  // A process-local recovery quarantine is not cleared by
  // a generic recovery request.
  owner.quarantine();

  assert(!owner.begin_new_disarmed_run(true, true));

  auto quarantined = owner.tick(
      4.0, true, true, false,
      evidence(4.0), guidance(4.0));

  assert(!quarantined.motion.permitted);
  assert(!owner.hardware_command_emitted());

  std::cout
      << "PASS: Native C++17 single-owner integration tests\n";
}
