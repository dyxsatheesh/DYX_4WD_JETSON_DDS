#include "backend_live_fixture.hpp"
#include "backend_api_fixture.hpp"

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
  NativeMissionCommandGateway gateway;
  NativeBackendMissionApi api(gateway);
  NativeBackendLiveStatus live(gateway);

  gateway_load(gateway);
  gateway_feed(gateway, 0.0);

  check(live.observe_tick(gateway_tick(gateway, 0.0)),
        "Initial safety tick accepted");

  const auto started = api.handle(
      api_req(kBackendMissionStartPath, 1, 0.1));

  check(started.http_status == 200,
        "Authorized START accepted");

  gateway_feed(gateway, 0.2);
  check(live.observe_tick(gateway_tick(gateway, 0.2)),
        "Approaching safety tick accepted");

  const auto healthy = live.handle(backend_live_req(0.21));

  check(healthy.snapshot_fresh &&
            healthy.stage == MissionStage::approaching,
        "Live status reports APPROACHING");
  check(healthy.rtk_status == "FIXED",
        "Live status reports RTK FIXED");

  std::cout << "=== RTK FLOAT ===\n";

  gateway_feed(gateway, 1.0, 5);

  const auto floating_tick = gateway_tick(gateway, 1.0);

  check(!floating_tick.decision.safety.motion_allowed,
        "FLOAT immediately inhibits autonomous motion");
  check(floating_tick.decision.mission.stage ==
            MissionStage::paused,
        "FLOAT pauses native mission");

  check(live.observe_tick(floating_tick),
        "FLOAT safety tick accepted by live status");

  const auto floating = live.handle(backend_live_req(1.01));

  check(floating.snapshot_fresh,
        "FLOAT telemetry snapshot is fresh");
  check(floating.rtk_status == "FLOAT",
        "Backend reports FLOAT");
  check(floating.stage == MissionStage::paused,
        "Backend reports PAUSED");
  check(floating.safety_hold_fresh &&
            floating.safety_hold_reason ==
                "RTK_FLOAT_CONFIRMING",
        "Backend reports precise FLOAT hold reason");
  check(!floating.hardware_command_emitted,
        "Live-status endpoint is non-actuating");

  const auto status = api.handle(api_status_req());

  check(status.http_status == 200 &&
            status.stage == MissionStage::paused,
        "Mission API agrees on PAUSED");

  const auto denied = api.handle(
      api_req(kBackendMissionResumePath, 2, 1.02));

  check(denied.http_status != 200,
        "Backend rejects RESUME during FLOAT");

  std::cout << "=== RTK FIXED RECOVERY ===\n";

  gateway_feed(gateway, 2.0, 6);

  const auto fixed_tick = gateway_tick(gateway, 2.0);

  check(fixed_tick.decision.safety.motion_allowed,
        "FIXED restores safety eligibility");
  check(fixed_tick.decision.mission.stage ==
            MissionStage::paused,
        "FIXED does not automatically RESUME");

  check(live.observe_tick(fixed_tick),
        "FIXED safety tick accepted by live status");

  const auto recovered = live.handle(backend_live_req(2.01));

  check(recovered.rtk_status == "FIXED",
        "Backend reports recovered RTK FIXED");
  check(recovered.stage == MissionStage::paused,
        "Backend still reports PAUSED");

  const auto resumed = api.handle(
      api_req(kBackendMissionResumePath, 3, 2.02));

  check(resumed.http_status == 200,
        "Explicit authorized RESUME accepted");

  gateway_feed(gateway, 2.1);

  check(live.observe_tick(gateway_tick(gateway, 2.1)),
        "Post-RESUME safety tick accepted");

  const auto approaching = live.handle(backend_live_req(2.11));

  check(approaching.stage == MissionStage::approaching,
        "Backend reports APPROACHING after manual RESUME");

  std::cout << "=== STALE STATUS AND STOP ===\n";

  const auto stale = live.handle(backend_live_req(3.0));

  check(!stale.snapshot_fresh &&
            stale.code == "LIVE_STATUS_STALE_OR_FAULT",
        "Expired live snapshot marked stale");
  check(!stale.safety_hold_fresh &&
            stale.rtk_status == "UNAVAILABLE",
        "Stale snapshot does not display trusted RTK FIX");

  auto stop_request = api_req(
      kBackendMissionStopPath, 4, -1.0);
  stop_request.principal.owns_control_lease = false;

  const auto stopped = api.handle(stop_request);

  check(stopped.http_status == 200,
        "Authenticated STOP accepted without lease");
  check(stopped.stage == MissionStage::stopped,
        "Backend reports STOPPED");
  check(!gateway.emits_hardware_commands(),
        "Gateway remains non-actuating");

  std::cout
      << "ALL H8H BACKEND SAFETY SYNC TESTS PASSED ("
      << checks << " checks)\n";
}
