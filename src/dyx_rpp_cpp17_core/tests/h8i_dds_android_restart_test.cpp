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

  std::cout << "=== NORMAL OPERATION ===\n";

  gateway_load(gateway);
  gateway_feed(gateway, 100.0);

  check(live.observe_tick(gateway_tick(gateway, 100.0)),
        "Initial safety tick accepted");

  const auto initial = stream.connect(stream_read(100.01));

  check(initial.delivered && initial.full_resync,
        "Initial full SSE snapshot delivered");

  auto view = android.accept(initial, 100.01);

  check(view.connected && view.snapshot_fresh && view.rtk_fixed,
        "Android initially displays fresh FIXED");

  check(api.handle(
      api_req(kBackendMissionStartPath, 1, 100.02)).http_status == 200,
      "Mission START accepted");

  gateway_feed(gateway, 100.10);

  check(live.observe_tick(gateway_tick(gateway, 100.10)),
        "Running mission safety tick accepted");

  auto event = stream.poll(stream_read(100.11));

  check(event.delivered, "Running mission status delivered");

  view = android.accept(event, 100.11);

  check(view.connected && view.rtk_fixed,
        "Android receives running telemetry");

  std::cout << "=== PX4 TELEMETRY EPOCH FAILURE ===\n";

  const auto rollback = gateway.observe_vehicle({
      gateway_stamp(100.0),
      100.20,
      2, 14, 3, false, 0
  });

  check(rollback == DdsTelemetryDecision::px4_epoch_changed,
        "Backward PX4 timestamp detected");

  check(!gateway.has_active_mission(),
        "PX4 epoch fault terminates mission");

  check(gateway.safe_stop_requested(),
        "PX4 epoch fault requests safe STOP");

  const auto fault_tick = gateway_tick(gateway, 100.20);

  check(fault_tick.telemetry_fault_latched,
        "Telemetry integrity fault remains latched");

  check(!fault_tick.preview_forward_permitted,
        "Forward preview disabled after epoch failure");

  check(live.observe_tick(fault_tick),
        "Faulted telemetry tick accepted by live status");

  event = stream.poll(stream_read(100.21));

  check(event.delivered,
        "Faulted backend status delivered to SSE stream");

  view = android.accept(event, 100.21);

  check(!view.snapshot_fresh && !view.rtk_fixed,
        "Android clears FIXED after telemetry fault");

  check(!view.mission_commands_enabled,
        "Faulted telemetry cannot authorize motion");

  std::cout << "=== STREAM DISCONNECTION ===\n";

  view = android.on_connection_lost();
  stream.disconnect();

  check(!view.connected && !view.rtk_fixed,
        "Android disconnect invalidates FIXED");

  check(view.requires_resync,
        "Disconnect requires a new full snapshot");

  check(!gateway.has_active_mission(),
        "Disconnect cannot restore stopped mission");

  std::cout << "=== AUTHORIZED STREAM EPOCH RESET ===\n";

  check(!stream.reset_epoch_disarmed(
            false, true, 802, 100.30),
        "Unauthorized stream epoch reset rejected");

  check(!stream.reset_epoch_disarmed(
            true, false, 802, 100.30),
        "Armed stream epoch reset rejected");

  check(!stream.reset_epoch_disarmed(
            true, true, 801, 100.30),
        "Old stream epoch rejected");

  check(stream.reset_epoch_disarmed(
            true, true, 802, 100.30),
        "Authorized DISARMED stream epoch reset accepted");

  check(!android.recover_disarmed(
            false, true, 802, 100.30),
        "Android unauthorized recovery rejected");

  check(!android.recover_disarmed(
            true, false, 802, 100.30),
        "Android armed recovery rejected");

  check(android.recover_disarmed(
            true, true, 802, 100.30),
        "Android authorized DISARMED recovery accepted");

  const auto recovered = stream.connect(
      stream_read(100.31), view.last_event_id);

  check(recovered.delivered && recovered.full_resync,
        "New stream epoch requires full snapshot");

  check(recovered.event_id == "802-1",
        "New service epoch starts with fresh sequence");

  view = android.accept(recovered, 100.31);

  check(view.connected && view.requires_resync == false,
        "Android accepts new full snapshot");

  check(!view.snapshot_fresh && !view.rtk_fixed,
        "New stream does not conceal telemetry fault");

  check(!view.mission_commands_enabled,
        "Recovered Android stream grants no motion authority");

  check(!gateway.has_active_mission(),
        "Stream recovery cannot restart mission");

  const auto old_resume = api.handle(
      api_req(kBackendMissionResumePath, 2, 100.32));

  check(old_resume.http_status != 200,
        "Old mission RESUME remains rejected");

  auto stop = api_req(kBackendMissionStopPath, 3, -1.0);
  stop.principal.owns_control_lease = false;

  check(api.handle(stop).http_status == 200,
        "Authenticated STOP remains available");

  check(!gateway.emits_hardware_commands(),
        "Gateway remains non-actuating");

  check(!view.hardware_command_emitted &&
            !view.network_io_performed,
        "Android replica remains non-actuating");

  std::cout
      << "ALL H8I DDS AND ANDROID RESTART TESTS PASSED ("
      << checks << " checks)\n";
}
