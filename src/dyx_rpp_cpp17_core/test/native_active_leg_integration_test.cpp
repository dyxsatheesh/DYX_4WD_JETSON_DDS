#include "dyx_rpp_cpp17_core/native_control_owner.hpp"

#include <cassert>
#include <iostream>
#include <vector>

using namespace dyx::rpp;

int main() {
  NativeControlOwner owner;
  MissionLegPreviewInput sample{};
  sample.now_sec = 1.0;

  // No mission may yield usable guidance.
  auto result = owner.preview_active_leg(sample);
  assert(!result.valid);
  assert(result.hold_zero);
  assert(result.reason == "NO_ACTIVE_PREPARED_MISSION");

  const std::vector<Point2D> marking{
      {0.0, 0.0},
      {4.0, 0.0},
      {4.0, 4.0}
  };

  assert(owner.load_prepared(
      "active_leg_run_001",
      "active_leg_session_001",
      marking,
      MissionMode::automatic,
      true, true, true
  ) == ClientCommandStatus::accepted);

  assert(owner.mission().stage() == MissionStage::ready);

  // READY must not generate active motion guidance.
  result = owner.preview_active_leg(sample);
  assert(!result.valid);
  assert(result.hold_zero);
  assert(result.reason == "MISSION_NOT_APPROACHING");

  assert(owner.prepared_mission() != nullptr);
  const auto& goals = owner.prepared_mission()->marking_goals;

  assert(goals.size() == 3);
  assert(goals[0].goal_label == "P0001");
  assert(goals[1].goal_label == "P0002");
  assert(goals[2].goal_label == "P0003");

  // Each marking point is bound to an actual path index.
  assert(goals[0].raw_path_index == 0);
  assert(goals[1].raw_path_index > goals[0].raw_path_index);
  assert(goals[2].raw_path_index > goals[1].raw_path_index);

  // Verify the existing RPP guidance can be constructed for P2
  // from the actual prepared trajectory, not a duplicate path.
  MissionLegPreview p2(
      owner.prepared_mission()->snapshot.raw_navigation(),
      goals[1].raw_path_index);

  assert(p2.goal_identity() == "P0002");
  assert(!p2.latched());

  // No calibrated marking reference: immediate safe hold.
  result = p2.step(sample);
  assert(!result.valid);
  assert(result.hold_zero);
  assert(p2.latched());

  // Once latched, there is no automatic recovery.
  sample.marking_reference_calibrated = true;
  result = p2.step(sample);
  assert(!result.valid);
  assert(result.hold_zero);

  owner.force_stop();

  result = owner.preview_active_leg(sample);
  assert(!result.valid);
  assert(result.hold_zero);

  std::cout
      << "PASS: Active mission/RPP guidance integration\n";
}
