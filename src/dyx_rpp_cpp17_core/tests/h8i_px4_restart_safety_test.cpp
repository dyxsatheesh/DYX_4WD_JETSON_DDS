#include "backend_api_fixture.hpp"

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

int main() {
  std::cout << "=== INITIAL MISSION ===\n";

  NativeMissionCommandGateway gateway;
  NativeBackendMissionApi api(gateway);

  gateway_load(gateway);

  gateway_feed(gateway, 0.0);
  gateway_tick(gateway, 0.0);

  const auto started = api.handle(
      api_req(kBackendMissionStartPath, 1, 0.1));

  check(started.http_status == 200,
        "Healthy mission START accepted");

  gateway_feed(gateway, 0.2);
  const auto running = gateway_tick(gateway, 0.2);

  check(running.preview_forward_permitted,
        "Healthy mission permits forward preview");

  std::cout << "=== PX4 EPOCH CHANGE ===\n";

  const auto epoch_change = gateway.observe_vehicle({
      gateway_stamp(0.1), 0.3,
      2, 14, 3, false, 0
  });

  check(epoch_change == DdsTelemetryDecision::px4_epoch_changed,
        "Backward PX4 timestamp detects epoch change");

  check(gateway.safe_stop_requested(),
        "Gateway requests safe STOP");

  check(!gateway.has_active_mission(),
        "Active mission terminated immediately");

  check(gateway.mission().stage() == MissionStage::stopped,
        "Mission enters STOPPED stage");

  check(!gateway.emits_hardware_commands(),
        "Epoch handling remains non-actuating");

  const auto status = api.handle(api_status_req());

  check(status.http_status == 200 &&
            status.stage == MissionStage::stopped,
        "Backend reports STOPPED after PX4 epoch change");

  const auto rejected = api.handle(
      api_req(kBackendMissionResumePath, 2, 0.31));

  check(rejected.http_status != 200,
        "Old mission RESUME rejected");

  std::cout << "=== CONTROLLED EPOCH RECOVERY ===\n";

  check(!gateway.begin_new_disarmed_px4_epoch(false, true),
        "Unauthorized epoch reset rejected");

  check(!gateway.begin_new_disarmed_px4_epoch(true, false),
        "Armed epoch reset rejected");

  check(gateway.begin_new_disarmed_px4_epoch(true, true),
        "Authorized DISARMED epoch reset accepted");

  check(!gateway.has_active_mission(),
        "Epoch reset does not restore old mission");

  check(gateway.mission().stage() == MissionStage::stopped,
        "Mission remains STOPPED after reset");

  const auto reused = gateway.load(
      gateway_run,
      gateway_session,
      {{0, 0, "P0001", "RUN:OLD:P0001"}},
      MissionMode::automatic,
      true, true, true);

  check(reused == ClientCommandStatus::run_id_reused,
        "Previous mission-run ID cannot be reused");

  const auto new_run = gateway.load(
      "run_H8I_new_0001",
      gateway_session,
      {{0, 0, "P0001", "RUN:NEW:P0001"}},
      MissionMode::automatic,
      true, true, true);

  check(new_run == ClientCommandStatus::accepted,
        "Separate authorized new mission can be loaded");

  check(gateway.mission().stage() == MissionStage::ready,
        "New mission is READY but not started");

  check(!gateway.dispatch(
      gateway_request(ClientMissionAction::start, 3, 0.4)).accepted(),
        "Old run identity cannot start new mission");

  gateway.force_stop();

  check(gateway.mission().stage() == MissionStage::stopped,
        "Independent force STOP accepted");

  std::cout << "=== RECOVERY QUARANTINE ===\n";

  NativeMissionCommandGateway quarantined;
  quarantined.latch_recovery_quarantine();

  check(quarantined.recovery_quarantined(),
        "Recovery quarantine latched");

  check(quarantined.safe_stop_requested(),
        "Quarantine requests safe STOP");

  check(!quarantined.begin_new_disarmed_px4_epoch(true, true),
        "Telemetry reset cannot bypass quarantine");

  check(quarantined.dispatch(
      gateway_request(ClientMissionAction::resume, 1, 1.0))
            .status == ClientCommandStatus::recovery_quarantined,
        "Manual RESUME blocked by quarantine");

  auto stop = gateway_request(
      ClientMissionAction::stop, 2, -1.0);
  stop.authorized_control_lease = false;

  check(quarantined.dispatch(stop).accepted(),
        "Authenticated STOP remains available");

  check(!quarantined.emits_hardware_commands(),
        "Quarantine test remains non-actuating");

  std::cout
      << "ALL H8I PX4 RESTART SAFETY TESTS PASSED ("
      << checks << " checks)\n";
}
