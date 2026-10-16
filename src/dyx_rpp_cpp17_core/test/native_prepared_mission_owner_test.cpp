#include "dyx_rpp_cpp17_core/native_control_owner.hpp"

#include <cassert>
#include <iostream>
#include <vector>

using namespace dyx::rpp;

int main() {
  NativeControlOwner owner;

  const std::vector<Point2D> points{
      {0.0, 0.0},
      {4.0, 0.0},
      {4.0, 4.0}
  };

  // Unauthorized loading must fail.
  assert(owner.load_prepared(
      "native_run_001", "native_session_001", points,
      MissionMode::automatic,
      false, true, true) == ClientCommandStatus::unauthorized);

  assert(owner.prepared_mission() == nullptr);

  // A trusted adapter may load; this test supplies synthetic
  // attestations only and does NOT demonstrate authentication.
  assert(owner.load_prepared(
      "native_run_001", "native_session_001", points,
      MissionMode::automatic,
      true, true, true) == ClientCommandStatus::accepted);

  const auto* prepared = owner.prepared_mission();

  assert(prepared != nullptr);
  assert(owner.has_active_mission());
  assert(owner.mission().stage() == MissionStage::ready);

  assert(prepared->marking_goals.size() == 3);
  assert(prepared->snapshot.marking_points.size() == 3);
  assert(prepared->geometry.raw_points.size() > 3);

  assert(prepared->snapshot.marking_points[0].x == 0.0);
  assert(prepared->snapshot.marking_points[1].x == 4.0);
  assert(prepared->snapshot.marking_points[2].y == 4.0);

  // Duplicate installation cannot replace an active mission.
  assert(owner.load_prepared(
      "native_run_002", "native_session_002", points,
      MissionMode::automatic,
      true, true, true) == ClientCommandStatus::invalid_stage);

  // Loading is READY only; no START and no motor output.
  assert(!owner.hardware_command_emitted());

  owner.force_stop();
  assert(!owner.has_active_mission());
  assert(!owner.hardware_command_emitted());

  std::cout
      << "PASS: Native trajectory-to-mission owner integration\\n";
}
