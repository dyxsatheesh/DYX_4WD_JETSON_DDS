#include "backend_android_draft_codec_fixture.hpp"
static std::string replace_exact(std::string input,const std::string& a,const std::string& b) {
  const auto i=input.find(a);assert(i!=std::string::npos);
  input.replace(i,a.size(),b);return input;
}
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire server(backend);
  NativeOfflineAndroidDraftHttpCodec codec;
  const auto p=codec.encode(codec_proposal(kBackendMissionUploadPath,1));
  auto reply=codec_verified_response(server.handle(codec_trusted_http(p)),p);
  assert(codec.decode(reply,p).valid);
  const auto original=reply.wire.body;
  for (const auto& bad : {
       replace_exact(original,"\"ok\":true","\"ok\":false"),
       replace_exact(original,"\"loaded\":false","\"loaded\":true"),
       replace_exact(original,"\"ready\":true","\"ready\":false"),
       replace_exact(original,"\"preview_only\":true","\"preview_only\":false"),
       replace_exact(original,"\"decision_only\":true","\"decision_only\":false"),
       replace_exact(original,"\"hardware_command_emitted\":false","\"hardware_command_emitted\":true"),
       replace_exact(original,"\"revision\":1","\"revision\":0"),
       replace_exact(original,"\"revision\":1","\"revision\":18446744073709551616"),
       replace_exact(original,"\"marking_count\":3","\"marking_count\":0"),
       replace_exact(original,"\"code\":\"UPLOAD_PREVIEW_READY\"","\"code\":\"MISSION_LOADED_READY_NOT_STARTED\""),
       replace_exact(original,"\"ok\":true","\"ok\":true,\"role\":\"admin\""),
       replace_exact(original,"\"ok\":true","\"ok\":true,\"ok\":true") }) {
    reply.wire.body=bad;
    assert(!codec.decode(reply,p).valid);
  }
  reply.wire.body=original;
  assert(codec.decode(reply,p).valid);
  assert(!gateway.has_active_mission());
}
