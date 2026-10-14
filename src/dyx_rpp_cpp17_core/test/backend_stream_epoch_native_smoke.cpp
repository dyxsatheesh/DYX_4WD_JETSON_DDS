#include "backend_stream_fixture.hpp"
#include <limits>
int main() {
  NativeMissionCommandGateway g;
  NativeBackendMissionApi api(g);
  NativeBackendLiveStatus live(g);
  NativeBackendHttpWire wire(api,live);
  stream_setup(g,live);
  NativeOfflineLiveStream stream(wire,200);
  assert(stream.connect(stream_read(100.1)).delivered);
  const auto rollback=stream.poll(stream_read(99.0));
  assert(rollback.code=="STREAM_HOST_CLOCK_FAULT");
  assert(stream.clock_fault_latched() && !stream.connected());
  assert(stream.connect(stream_read(101)).code=="STREAM_CLOCK_FAULT_LATCHED");
  assert(!stream.reset_epoch_disarmed(false,true,201,102));
  assert(!stream.reset_epoch_disarmed(true,false,201,102));
  assert(!stream.reset_epoch_disarmed(true,true,200,102));
  assert(!stream.reset_epoch_disarmed(true,true,201,std::numeric_limits<double>::quiet_NaN()));
  assert(stream.reset_epoch_disarmed(true,true,201,102));
  assert(!stream.clock_fault_latched() && !stream.connected());
  const auto recovered=stream.connect(stream_read(102.01),"200-1");
  assert(recovered.delivered && recovered.full_resync);
  assert(recovered.code=="STREAM_CURSOR_RESYNC_REQUIRED");
  assert(recovered.event_id=="201-1");
  // These are offline decision objects, not stream socket or hardware actions.
  assert(!recovered.network_io_performed && !recovered.hardware_command_emitted);
}
