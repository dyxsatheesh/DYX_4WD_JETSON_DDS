#include "backend_android_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionApi api(gateway);
  NativeBackendLiveStatus live(gateway);
  NativeBackendHttpWire wire(api,live);
  stream_setup(gateway,live);
  NativeOfflineLiveStream stream(wire,450);
  NativeOfflineAndroidTelemetryReplica android;
  assert(android.accept(stream.connect(stream_read(100.01)),100.01).rtk_fixed);
  const auto lost=android.on_connection_lost();
  assert_android_disconnected(lost);
  assert(android.last_event_id_hint()=="450-1");
  assert_android_disconnected(android.on_monotonic_tick(100.02));
  stream.disconnect();
  const auto recovery=stream.connect(stream_read(100.03),android.last_event_id_hint());
  assert(recovery.full_resync);
  const auto resumed=android.accept(recovery,100.03);
  assert(resumed.connected && resumed.rtk_fixed && resumed.last_event_id=="450-2");
  // Replaying an old event after successful reconnection must clear cached FIX.
  assert_android_disconnected(android.accept(recovery,100.04));
  // Even a sequence-consistent event cannot restore FIX without a full resync.
  const auto next=stream.poll(stream_read(100.72));
  assert(next.delivered && !next.full_resync);
  assert_android_disconnected(android.accept(next,100.72));
  stream.disconnect();
  const auto full=stream.connect(stream_read(100.75),android.last_event_id_hint());
  const auto accepted=android.accept(full,100.75);
  assert(accepted.connected && !accepted.snapshot_fresh && !accepted.rtk_fixed);
  assert_android_safe(accepted);
}
