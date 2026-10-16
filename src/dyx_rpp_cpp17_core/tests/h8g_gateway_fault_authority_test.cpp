#include "mission_command_gateway_fixture.hpp"

#include <cstdlib>
#include <iostream>

using namespace dyx::rpp;

static int checks = 0;

static void check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
  ++checks;
  std::cout << "PASS: " << message << '\n';
}

static void start_healthy(NativeMissionCommandGateway& g) {
  gateway_load(g);
  gateway_feed(g, 0.0);
  check(gateway_tick(g, 0.0).decision.mission.stage ==
            MissionStage::ready,
        "Healthy mission READY");
  check(g.dispatch(
            gateway_request(ClientMissionAction::start, 1, 0.1))
            .accepted(),
        "Healthy START accepted");

  gateway_feed(g, 0.2);
  const auto running = gateway_tick(g, 0.2);
  check(running.preview_forward_permitted,
        "Healthy mission permits forward preview");
}

static void stale_telemetry() {
  std::cout << "=== STALE TELEMETRY ===\n";

  NativeMissionCommandGateway g;
  start_healthy(g);

  // Do not refresh telemetry; advance the host monotonic clock.
  const auto stale = gateway_tick(g, 3.0);

  check(!stale.decision.safety.motion_allowed,
        "Expired DDS telemetry inhibits motion");
  check(stale.decision.mission.stage == MissionStage::paused,
        "Expired telemetry pauses mission");
  check(!stale.preview_forward_permitted,
        "Expired telemetry denies forward preview");

  const auto resume = g.dispatch(
      gateway_request(ClientMissionAction::resume, 2, 3.1));
  check(!resume.accepted(),
        "RESUME rejected with stale telemetry");

  const auto stop = g.dispatch(
      gateway_request(ClientMissionAction::stop, 3, -1.0));
  check(stop.accepted(),
        "STOP accepted despite stale telemetry");
  check(!g.emits_hardware_commands(),
        "Stale telemetry test remains non-actuating");
}

static void lost_rtk() {
  std::cout << "=== LOST RTK FIX ===\n";

  NativeMissionCommandGateway g;
  start_healthy(g);

  gateway_feed(g, 1.0, 4);
  const auto lost = gateway_tick(g, 1.0);

  check(lost.decision.safety.hold == SafetyHold::rtk_lost,
        "Degraded GNSS fix classified as RTK loss");
  check(!lost.decision.safety.motion_allowed,
        "Lost RTK inhibits motion");
  check(lost.decision.mission.stage == MissionStage::paused,
        "Lost RTK pauses mission");
  check(!lost.preview_forward_permitted,
        "Lost RTK denies preview");

  gateway_feed(g, 2.0, 6);
  const auto fixed = gateway_tick(g, 2.0);

  check(fixed.decision.mission.stage == MissionStage::paused,
        "FIXED does not auto-resume RTK-loss mission");
  check(!fixed.preview_forward_permitted,
        "No forward preview after unacknowledged RTK loss");

  const auto denied = g.dispatch(
      gateway_request(ClientMissionAction::resume, 2, 2.1));
  check(!denied.accepted(),
        "Manual safety acknowledgment still required");

  const auto stop = g.dispatch(
      gateway_request(ClientMissionAction::stop, 3, -1.0));
  check(stop.accepted(),
        "STOP accepted during RTK recovery hold");
}

static void authorization_and_estop() {
  std::cout << "=== AUTHORIZATION AND EMERGENCY STOP ===\n";

  NativeMissionCommandGateway g;
  gateway_load(g);
  gateway_feed(g, 0.0);
  gateway_tick(g, 0.0);

  auto unauth =
      gateway_request(ClientMissionAction::start, 1, 0.1);
  unauth.authenticated_operator = false;

  check(g.dispatch(unauth).status ==
            ClientCommandStatus::unauthorized,
        "Unauthenticated START rejected");

  auto no_lease =
      gateway_request(ClientMissionAction::start, 2, 0.1);
  no_lease.authorized_control_lease = false;

  check(g.dispatch(no_lease).status ==
            ClientCommandStatus::unauthorized,
        "START without control lease rejected");

  check(g.dispatch(
            gateway_request(ClientMissionAction::start, 3, 0.1))
            .accepted(),
        "Authorized START accepted");

  gateway_feed(g, 1.0);
  const auto estop = gateway_tick(g, 1.0, true, true);

  check(!estop.decision.safety.motion_allowed,
        "Emergency stop inhibits motion");
  check(estop.decision.mission.stage == MissionStage::fault,
        "Emergency stop faults mission");
  check(!estop.preview_forward_permitted,
        "Emergency stop denies preview");

  auto blocked =
      gateway_request(ClientMissionAction::resume, 4, 1.1);
  check(!g.dispatch(blocked).accepted(),
        "RESUME rejected during emergency stop");

  auto stop =
      gateway_request(ClientMissionAction::stop, 5, -1.0);
  stop.authorized_control_lease = false;

  check(g.dispatch(stop).accepted(),
        "Authenticated STOP remains available");
  check(!g.has_active_mission(),
        "Emergency STOP clears active mission");
  check(!g.emits_hardware_commands(),
        "Emergency scenario remains non-actuating");
}

int main() {
  stale_telemetry();
  lost_rtk();
  authorization_and_estop();

  std::cout
      << "ALL H8G-2 SAFETY TESTS PASSED ("
      << checks << " checks)\n";
}
