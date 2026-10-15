#include "backend_android_draft_codec_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire server(backend);
  NativeOfflineAndroidDraftHttpCodec codec;
  auto upload=codec.encode(codec_proposal(kBackendMissionUploadPath,1));
  assert(upload.valid && !upload.hardware_command_emitted && !upload.network_io_performed);
  assert(upload.body.find("\"marking_points\"")!=std::string::npos);
  const auto upload_wire=server.handle(codec_trusted_http(upload));
  assert(upload_wire.status==200);
  auto reply=codec.decode(codec_verified_response(upload_wire,upload),upload);
  assert(reply.valid && reply.backend.code=="UPLOAD_PREVIEW_READY");
  assert(reply.backend.revision==1 && reply.backend.marking_count==3);
  assert(!reply.hardware_command_emitted && !gateway.has_active_mission());
  const auto rev1=codec_signature(backend);
  auto update=codec.encode(codec_proposal(kBackendMissionUpdatePath,2,1,rev1));
  assert(update.valid && update.body.find(rev1)!=std::string::npos);
  const auto update_wire=server.handle(codec_trusted_http(update,11.0));
  auto edit_reply=codec.decode(codec_verified_response(update_wire,update,11.0,11.1),update);
  assert(edit_reply.valid && edit_reply.backend.revision==2);
  const auto rev2=codec_signature(backend);
  auto load=codec.encode(codec_proposal(kBackendMissionLoadPath,3,2,rev2));
  assert(load.valid && load.body.find("marking_points")==std::string::npos);
  const auto load_wire=server.handle(codec_trusted_http(load,12.0));
  auto load_reply=codec.decode(codec_verified_response(load_wire,load,12.0,12.1),load);
  assert(load_reply.valid && load_reply.backend.loaded && !load_reply.backend.preview_only);
  assert(gateway.has_active_mission() && gateway.mission().stage()==MissionStage::ready);
  assert(!gateway.emits_hardware_commands());
}
