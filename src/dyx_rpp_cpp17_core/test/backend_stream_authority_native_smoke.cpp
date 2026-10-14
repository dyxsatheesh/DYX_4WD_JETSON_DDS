#include "backend_stream_fixture.hpp"
int main() {
  NativeMissionCommandGateway g;
  NativeBackendMissionApi api(g);
  NativeBackendLiveStatus live(g);
  NativeBackendHttpWire wire(api,live);
  stream_setup(g,live);
  NativeOfflineLiveStream stream(wire,74);
  auto wrong=stream_read(100.01);
  wrong.principal.authenticated=false;
  assert(!stream.connect(wrong,"74-777").delivered);
  assert(!stream.connected() && stream.last_event_sequence()==0);
  wrong=stream_read(100.02);
  wrong.principal.can_read_mission=false;
  assert(!stream.connect(wrong).delivered);
  assert(!stream.connected());
  auto good=stream_read(100.03);
  assert(stream.connect(good).delivered);
  auto stolen=stream_read(100.04);
  stolen.principal.operator_session_id="SESSION_OTHER_READER";
  const auto busy=stream.connect(stolen,"74-1");
  assert(busy.code=="STREAM_SUBSCRIBER_BUSY" && !busy.delivered);
  assert(stream.connected());
  const auto denied=stream.poll(stolen);
  assert(!denied.delivered && denied.frame.empty());
  assert(denied.code=="STREAM_SUBSCRIBER_MISMATCH" && stream.connected());
  assert(stream.poll(stream_read(100.05)).code=="STREAM_UNCHANGED");
  wrong=stream_read(100.06);
  wrong.principal.can_read_mission=false;
  assert(stream.poll(wrong).code=="STREAM_SESSION_REVOKED");
  assert(!stream.connected());
  assert(stream.poll(stream_read(100.07)).code=="STREAM_NOT_CONNECTED");
  assert(stream.connect(stream_read(100.08)).delivered);
  wrong=stream_read(100.09);
  wrong.method="POST";
  assert(!stream.poll(wrong).delivered && !stream.connected());
  assert(g.mission().stage()==MissionStage::ready);
}
