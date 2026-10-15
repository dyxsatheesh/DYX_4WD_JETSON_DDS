#include "backend_android_draft_codec_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire server(backend);
  NativeOfflineAndroidDraftHttpCodec codec;
  const auto p=codec.encode(codec_proposal(kBackendMissionUploadPath,1));
  const auto wire=server.handle(codec_trusted_http(p));
  auto r=codec_verified_response(wire,p);
  assert(codec.decode(r,p).valid);
  r.verified_authenticated_transport=false;assert(!codec.decode(r,p).valid);
  r=codec_verified_response(wire,p);
  r.verified_request_sequence=2;assert(!codec.decode(r,p).valid);
  r=codec_verified_response(wire,p);
  r.verified_request_path=kBackendMissionUpdatePath;assert(!codec.decode(r,p).valid);
  r=codec_verified_response(wire,p);
  r.verified_sent_sec=20.0;r.verified_received_sec=19.0;
  assert(!codec.decode(r,p).valid);
  r=codec_verified_response(wire,p);
  r.verified_received_sec=25.0;assert(!codec.decode(r,p).valid);
  r=codec_verified_response(wire,p);
  r.wire.status=409;assert(!codec.decode(r,p).valid);
  r=codec_verified_response(wire,p);
  r.wire.content_type="text/html";assert(!codec.decode(r,p).valid);
  r=codec_verified_response(wire,p);
  r.wire.physical_command_emitted=true;assert(!codec.decode(r,p).valid);
  r=codec_verified_response(wire,p);
  r.wire.body += " ";assert(!codec.decode(r,p).valid);
  r=codec_verified_response(wire,p);
  auto wrong=codec.encode(codec_proposal(kBackendMissionUploadPath,2));
  assert(!codec.decode(r,wrong).valid);
  assert(!gateway.has_active_mission());
}
