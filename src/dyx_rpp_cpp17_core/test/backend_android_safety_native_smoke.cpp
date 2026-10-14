#include "backend_android_fixture.hpp"
#include <limits>
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionApi api(gateway);
  NativeBackendLiveStatus live(gateway);
  NativeBackendHttpWire wire(api,live);
  stream_setup(gateway,live);
  NativeOfflineLiveStream stream(wire,740);
  NativeOfflineAndroidTelemetryReplica android({3.0,32768});
  assert(android.accept(stream.connect(stream_read(100.01)),100.01).rtk_fixed);
  assert(android.on_monotonic_tick(102.9).rtk_fixed);
  assert_android_disconnected(android.on_monotonic_tick(103.02));
  const auto suspect = stream.poll(stream_read(103.03));
  // Nothing but a full authenticated fresh snapshot can recover display.
  if(suspect.delivered && !suspect.full_resync)
    assert_android_disconnected(android.accept(suspect,103.03));
  // Clock regressions latch; resync alone is never sufficient.
  assert_android_disconnected(android.on_monotonic_tick(102.0));
  assert(android.clock_fault_latched());
  stream.disconnect();
  assert_android_disconnected(android.accept(stream.connect(stream_read(103.10)),103.10));
  assert(!android.recover_disarmed(false,true,741,104));
  assert(!android.recover_disarmed(true,false,741,104));
  assert(!android.recover_disarmed(true,true,740,104));
  assert(!android.recover_disarmed(true,true,741,std::numeric_limits<double>::quiet_NaN()));
  assert(android.recover_disarmed(true,true,741,104));
  assert(stream.reset_epoch_disarmed(true,true,741,104));
  const auto restored=stream.connect(stream_read(104.01),"740-1");
  const auto state=android.accept(restored,104.01);
  assert(state.connected && state.last_event_id=="741-1");
  assert(!state.rtk_fixed); // telemetry is old; new epoch cannot revive stale FIX
  assert_android_safe(state);
  // The existing HTTP socket server remains explicitly read-only.
  const auto blocked=wire.handle(http_post(kBackendMissionStartPath,1,104.02));
  assert(blocked.status != 200 || !blocked.physical_command_emitted);
}
