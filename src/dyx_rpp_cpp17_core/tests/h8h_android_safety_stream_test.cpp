#include "backend_android_fixture.hpp"
#include "backend_api_fixture.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

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
  NativeBackendHttpWire wire(api, live);
  NativeOfflineLiveStream stream(wire, 801);
  NativeOfflineAndroidTelemetryReplica android;

  gateway_load(gateway);
  gateway_feed(gateway, 100.0);
  check(live.observe_tick(gateway_tick(gateway, 100.0)),
        "Initial DDS safety tick accepted");

  const auto first = stream.connect(stream_read(100.01));
  check(first.delivered && first.full_resync,
        "Initial SSE snapshot delivered");

  auto view = android.accept(first, 100.01);
  check(view.connected && view.snapshot_fresh && view.rtk_fixed,
        "Android displays fresh RTK FIXED");
  check(!view.mission_commands_enabled,
        "Android telemetry does not authorize commands");

  check(api.handle(
      api_req(kBackendMissionStartPath, 1, 100.02)).http_status == 200,
      "Backend START accepted independently");

  gateway_feed(gateway, 100.10);
  check(live.observe_tick(gateway_tick(gateway, 100.10)),
        "Running mission tick accepted");

  auto event = stream.poll(stream_read(100.11));
  check(event.delivered, "APPROACHING stream update delivered");

  view = android.accept(event, 100.11);
  check(view.status_json.find("\"stage\":\"approaching\"") !=
            std::string::npos ||
        view.status_json.find("\"stage\":\"APPROACHING\"") !=
            std::string::npos,
        "Android snapshot contains APPROACHING stage");

  std::cout << "=== FLOAT AND PAUSE ===\n";

  gateway_feed(gateway, 100.20, 5);
  const auto floating = gateway_tick(gateway, 100.20);

  check(!floating.decision.safety.motion_allowed &&
            floating.decision.mission.stage == MissionStage::paused,
        "FLOAT immediately pauses mission");

  check(live.observe_tick(floating),
        "FLOAT status update accepted");

  event = stream.poll(stream_read(100.21));
  check(event.delivered, "FLOAT event delivered");

  view = android.accept(event, 100.21);
  check(view.connected && view.snapshot_fresh && !view.rtk_fixed,
        "Android clears FIXED when RTK FLOAT arrives");
  check(view.status_json.find("\"rtk\":\"FLOAT\"") !=
            std::string::npos,
        "Android displays FLOAT from trusted backend");
  check(view.status_json.find("RTK_FLOAT_CONFIRMING") !=
            std::string::npos,
        "Stream carries native FLOAT safety diagnostic");
  check(!view.mission_commands_enabled,
        "FLOAT telemetry cannot enable Android commands");

  check(api.handle(
      api_req(kBackendMissionResumePath, 2, 100.22)).http_status != 200,
      "Backend rejects RESUME during FLOAT");

  std::cout << "=== FIXED RECOVERY ===\n";

  gateway_feed(gateway, 100.30, 6);
  const auto recovered = gateway_tick(gateway, 100.30);

  check(recovered.decision.mission.stage == MissionStage::paused,
        "FIXED recovery leaves mission PAUSED");
  check(live.observe_tick(recovered),
        "Recovery status accepted");

  event = stream.poll(stream_read(100.31));
  check(event.delivered, "FIXED recovery event delivered");

  view = android.accept(event, 100.31);
  check(view.rtk_fixed, "Android displays recovered FIXED");
  check(view.status_json.find("paused") != std::string::npos ||
        view.status_json.find("PAUSED") != std::string::npos,
        "Android still receives PAUSED mission state");
  check(!view.mission_commands_enabled,
        "FIXED display never grants motion authority");

  check(api.handle(
      api_req(kBackendMissionResumePath, 3, 100.32)).http_status == 200,
      "Manual RESUME accepted by backend");

  gateway_feed(gateway, 100.40);
  check(live.observe_tick(gateway_tick(gateway, 100.40)),
        "Post-RESUME mission tick accepted");

  event = stream.poll(stream_read(100.41));
  check(event.delivered, "Post-RESUME event delivered");

  view = android.accept(event, 100.41);
  check(view.connected && view.rtk_fixed,
        "Android displays fresh FIXED after manual RESUME");

  std::cout << "=== STALE STREAM ===\n";

  // No new DDS observations or safety ticks.
  event = stream.poll(stream_read(101.10));
  check(event.delivered,
        "Backend emits changed stale-status event");

  view = android.accept(event, 101.10);
  check(view.connected && !view.snapshot_fresh && !view.rtk_fixed,
        "Android clears FIXED on stale backend status");
  check(view.status_json.find("\"rtk\":\"UNAVAILABLE\"") !=
            std::string::npos,
        "Stale stream reports RTK UNAVAILABLE");

  std::cout << "=== DISCONNECTION ===\n";

  view = android.on_connection_lost();
  check(!view.connected && !view.snapshot_fresh &&
            !view.rtk_fixed && view.requires_resync,
        "Disconnect invalidates Android safety display");
  check(!view.mission_commands_enabled &&
            !view.hardware_command_emitted &&
            !view.network_io_performed,
        "Android replica remains read-only and non-actuating");

  check(!gateway.emits_hardware_commands(),
        "Gateway remains non-actuating");

  std::cout
      << "ALL H8H ANDROID STREAM TESTS PASSED ("
      << checks << " checks)\n";
}
