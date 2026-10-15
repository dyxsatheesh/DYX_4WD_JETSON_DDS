#include "backend_draft_http_framing_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  NativeOfflineDraftIngressLab ingress(wire);
  NativeOfflineDraftHttpFramingLab framing(ingress);
  NativeOfflineAndroidDraftCoordinator app;
  app.show(sync_screen());
  assert(framed_sync(app,framing,19.9,1).editor_resynchronized);
  auto request=app.begin_mutation(AndroidEditorIntent::upload,20.0);
  assert(request.valid);
  const auto good=framed_lab_request(request.method,request.path,request.body);
  auto reject=[&](const std::string& raw) {
    const auto outcome=framing.mutation(raw,request,lab_proof(20.02,2));
    assert(!outcome.reached_backend && !outcome.trusted_reply_provenance &&
           !outcome.network_io_performed && !outcome.hardware_command_emitted);
    assert(outcome.wire.status==400 && ingress.last_ingress_nonce()==1);
    assert(!backend.has_draft());
  };
  auto modified=good;
  const auto insert=modified.find("\r\n\r\n");
  modified.insert(insert,"\r\nContent-Length: 0"); reject(modified);
  modified=good;modified.insert(modified.find("\r\n\r\n"),"\r\nTransfer-Encoding: chunked");reject(modified);
  modified=good;modified.insert(modified.find("\r\n\r\n"),"\r\nAuthorization: Bearer fake");reject(modified);
  modified=good;modified.insert(modified.find("\r\n\r\n"),"\r\nX-Verified-PX4: true");reject(modified);
  modified=good;modified.insert(modified.find("\r\n\r\n"),"\r\nHost: offline.invalid");reject(modified);
  modified=good;modified.insert(modified.find("\r\n\r\n"),"\r\n folded");reject(modified);
  modified=good;modified.replace(0,4,"POST");modified.insert(modified.find(" HTTP/1.1")," ");reject(modified);
  modified=good+good;reject(modified);
  modified=good+"GET / HTTP/1.1\r\n\r\n";reject(modified);
  modified=good;modified.replace(modified.find("Content-Length: ")+16,
     std::to_string(request.body.size()).size(),"0001");reject(modified);
  modified=good;modified.replace(modified.find("Content-Length: ")+16,
     std::to_string(request.body.size()).size(),"999999999999999999999999");reject(modified);
  modified=good;modified.replace(modified.find("\r\n"),2,"\n");reject(modified);
  modified=good;modified.insert(modified.find("\r\n\r\n")+4,"x");reject(modified);
  // Valid request is not consumed by the invalid framing attempts.
  const auto ok=framing.mutation(good,request,lab_proof(20.02,2));
  assert(ok.reached_backend && ok.wire.status==200 && backend.has_draft());
  assert(!gateway.emits_hardware_commands());
}
