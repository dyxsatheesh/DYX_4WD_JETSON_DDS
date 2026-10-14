#include "backend_android_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionApi api(gateway);
  NativeBackendLiveStatus live(gateway);
  NativeBackendHttpWire wire(api,live);
  stream_setup(gateway, live);
  NativeOfflineLiveStream stream(wire,301);
  NativeOfflineAndroidTelemetryReplica android;
  const auto first = stream.connect(stream_read(100.01));
  const auto displayed = android.accept(first,100.01);
  assert(displayed.code=="ANDROID_FULL_SNAPSHOT");
  assert(displayed.connected && displayed.snapshot_fresh && displayed.rtk_fixed);
  assert(displayed.last_event_id=="301-1");
  assert(displayed.status_json.find("\"rtk\":\"FIXED\"")!=std::string::npos);
  assert_android_safe(displayed);
  const auto not_changed = stream.poll(stream_read(100.20));
  assert(not_changed.code=="STREAM_UNCHANGED");
  assert(android.on_monotonic_tick(100.2).rtk_fixed);

  // No more PX4 ticks: server must invalidate its own RTK state, and the
  // Android-facing replica must reflect it immediately.
  const auto stale = stream.poll(stream_read(100.72));
  const auto stale_view=android.accept(stale,100.72);
  assert(stale_view.connected && !stale_view.snapshot_fresh);
  assert(!stale_view.rtk_fixed && !stale_view.requires_resync);
  assert(stale_view.status_json.find("\"rtk\":\"UNAVAILABLE\"")!=std::string::npos);
  const auto heartbeat=stream.poll(stream_read(102.75));
  assert(heartbeat.heartbeat);
  const auto beat_view=android.accept(heartbeat,102.75);
  assert(beat_view.code=="ANDROID_HEARTBEAT" && !beat_view.rtk_fixed);
  assert_android_safe(beat_view);
}
