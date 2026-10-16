#include "mission_command_gateway_fixture.hpp"
#include <cstdlib>
#include <iostream>

using namespace dyx::rpp;

static int checks = 0;

static void check(bool ok, const char* message) {
  if (!ok) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
  ++checks;
  std::cout << "PASS: " << message << '\n';
}

int main() {
  NativeMissionCommandGateway g;

  gateway_load(g);
  gateway_feed(g, 0.0);

  auto ready = gateway_tick(g, 0.0);
  check(ready.decision.mission.stage == MissionStage::ready,
        "Healthy mission READY");

  auto start = g.dispatch(
      gateway_request(ClientMissionAction::start, 1, 0.1));
  check(start.accepted(), "Operator START accepted");

  gateway_feed(g, 0.2);
  auto running = gateway_tick(g, 0.2);

  check(running.decision.mission.stage == MissionStage::approaching,
        "Mission APPROACHING");
  check(running.preview_forward_permitted,
        "Healthy forward preview eligible");

  gateway_feed(g, 1.0, 5);
  auto floating = gateway_tick(g, 1.0);

  check(!floating.decision.safety.motion_allowed,
        "FLOAT immediately blocks autonomous motion");
  check(floating.decision.safety.hold ==
            SafetyHold::rtk_float_confirming,
        "Dedicated FLOAT confirmation diagnostic");
  check(floating.decision.mission.stage == MissionStage::paused,
        "FLOAT immediately pauses mission");
  check(!floating.preview_forward_permitted,
        "No forward preview during FLOAT");

  auto denied = g.dispatch(
      gateway_request(ClientMissionAction::resume, 2, 1.01));
  check(!denied.accepted(),
        "Operator RESUME rejected during FLOAT");

  gateway_feed(g, 2.0, 6);
  auto recovered = gateway_tick(g, 2.0);

  check(recovered.decision.safety.motion_allowed,
        "FIXED restores safety eligibility");
  check(recovered.decision.mission.stage == MissionStage::paused,
        "FIXED does not automatically RESUME");
  check(!recovered.preview_forward_permitted,
        "No preview before manual RESUME");
  check(!recovered.decision.auto_resumed_float,
        "Automatic RTK recovery disabled");

  auto replay = g.dispatch(
      gateway_request(ClientMissionAction::resume, 2, 2.01));
  check(replay.status ==
            ClientCommandStatus::duplicate_or_reordered,
        "Rejected command sequence cannot be replayed");

  auto resume = g.dispatch(
      gateway_request(ClientMissionAction::resume, 3, 2.1));
  check(resume.accepted(),
        "Fresh explicit operator RESUME accepted");

  gateway_feed(g, 2.2, 6);
  auto resumed = gateway_tick(g, 2.2);

  check(resumed.decision.mission.stage == MissionStage::approaching,
        "Mission APPROACHING after operator RESUME");
  check(resumed.preview_forward_permitted,
        "Forward preview restored only after RESUME");

  // A telemetry update revokes the previous safety tick.
  check(g.observe_obstacle({2.21, true}) ==
            DdsTelemetryDecision::accepted,
        "New telemetry invalidates prior authority");

  auto stale = g.dispatch(
      gateway_request(ClientMissionAction::skip, 4, 2.22));
  check(!stale.accepted(),
        "Command denied without new safety tick");

  auto stop_req =
      gateway_request(ClientMissionAction::stop, 5, -1.0);
  stop_req.authorized_control_lease = false;

  auto stopped = g.dispatch(stop_req);

  check(stopped.accepted(),
        "Authenticated STOP accepted despite stale safety");
  check(g.mission().stage() == MissionStage::stopped,
        "Mission STOPPED");
  check(!g.has_active_mission(),
        "No active mission after STOP");
  check(!g.emits_hardware_commands(),
        "Gateway remains non-actuating");

  std::cout
      << "ALL H8G GATEWAY RTK TESTS PASSED ("
      << checks << " checks)\n";
}
