#include "backend_stream_fixture.hpp"
int main() {
  NativeMissionCommandGateway g;
  NativeBackendMissionApi api(g);
  NativeBackendLiveStatus live(g);
  NativeBackendHttpWire wire(api,live);
  stream_setup(g,live);
  NativeOfflineLiveStream stream(wire,121);
  assert(stream.connect(stream_read(100.01)).event_id=="121-1");
  stream.disconnect();
  const auto reconnect=stream.connect(stream_read(100.02),"121-1");
  assert(reconnect.delivered && reconnect.full_resync);
  assert(reconnect.code=="STREAM_RECONNECTED_NO_REPLAY");
  assert(reconnect.event_id=="121-2");
  // Malformed, foreign-epoch and future IDs must not select old data or bypass
  // authentication. Every reconnection gets a new full snapshot.
  auto invalid=stream.connect(stream_read(100.03),"9999999999999999999999999-3");
  assert(invalid.code=="STREAM_CURSOR_RESYNC_REQUIRED");
  assert(invalid.event_id=="121-3");
  auto foreign=stream.connect(stream_read(100.04),"120-1");
  assert(foreign.code=="STREAM_CURSOR_RESYNC_REQUIRED");
  assert(foreign.event_id=="121-4");
  auto future=stream.connect(stream_read(100.05),"121-5");
  assert(future.code=="STREAM_CURSOR_RESYNC_REQUIRED");
  assert(future.event_id=="121-5");
  // Event IDs are not proof of transport delivery or actuator execution.
  assert(!future.network_io_performed && !future.hardware_command_emitted);
}
