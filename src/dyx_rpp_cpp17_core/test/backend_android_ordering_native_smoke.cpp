#include "backend_android_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionApi api(gateway);
  NativeBackendLiveStatus live(gateway);
  NativeBackendHttpWire wire(api,live);
  stream_setup(gateway,live);
  NativeOfflineLiveStream stream(wire,570);
  NativeOfflineAndroidTelemetryReplica android;
  const auto first=stream.connect(stream_read(100.01));
  assert(android.accept(first,100.01).rtk_fixed);
  auto injected=first;
  injected.frame.replace(0,3,"ID:"); // malformed SSE frame
  assert_android_disconnected(android.accept(injected,100.02));
  assert_android_disconnected(android.accept(first,100.03)); // replayed resync
  stream.disconnect();
  const auto full=stream.connect(stream_read(100.04),"570-1");
  assert(android.accept(full,100.04).rtk_fixed);
  // An impossible next event ID is a gap, not a permissible skipped snapshot.
  auto gap=stream.poll(stream_read(100.74));
  assert(gap.delivered);
  gap.event_id="570-999";
  gap.frame.replace(0,9,"id: 570-999");
  assert_android_disconnected(android.accept(gap,100.74));
  // Any client-generated mutation purporting to emit hardware commands fails.
  auto fabricated=gap;
  fabricated.hardware_command_emitted=true;
  assert_android_disconnected(android.accept(fabricated,100.75));
  // Unknown fields/malformed encodings are not equivalent to typed backend data.
  assert_android_safe(android.on_monotonic_tick(100.76));
}
