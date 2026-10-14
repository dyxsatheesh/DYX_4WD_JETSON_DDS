#include "backend_stream_fixture.hpp"
int main() {
  NativeMissionCommandGateway g;
  NativeBackendMissionApi api(g);
  NativeBackendLiveStatus live(g);
  NativeBackendHttpWire wire(api,live);
  stream_setup(g,live);
  NativeOfflineLiveStream stream(wire,17,{2.0,32768});
  const auto first=stream.connect(stream_read(100.01));
  assert(first.delivered && first.full_resync && !first.heartbeat);
  assert(first.event_id=="17-1" && first.frame.find("id: 17-1\nevent: rover.status\ndata: {")==0);
  assert(first.frame.find("\"rtk\":\"FIXED\"")!=std::string::npos);
  assert(!first.network_io_performed && !first.hardware_command_emitted);
  assert(stream.poll(stream_read(100.2)).code=="STREAM_UNCHANGED");
  // No PX4 tick; the previously fresh status must transition to STALE rather
  // than endlessly replaying a misleading FIXED snapshot.
  const auto stale=stream.poll(stream_read(100.7));
  assert(stale.delivered && stale.event_id=="17-2");
  assert(stale.frame.find("\"fresh\":false")!=std::string::npos);
  assert(stale.frame.find("\"rtk\":\"UNAVAILABLE\"")!=std::string::npos);
  assert(stream.poll(stream_read(100.8)).code=="STREAM_UNCHANGED");
  const auto ping=stream.poll(stream_read(102.71));
  assert(ping.delivered && ping.heartbeat && ping.event_id=="17-3");
  assert(ping.frame.find("\"fresh\":false")!=std::string::npos);
}
