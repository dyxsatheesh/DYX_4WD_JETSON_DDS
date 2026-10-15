#include "backend_draft_http_framing_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  NativeOfflineDraftIngressLab ingress(wire);
  NativeOfflineDraftHttpFramingLab framing(ingress);
  NativeOfflineAndroidDraftCoordinator app;
  app.show(sync_screen());
  assert(framed_sync(app,framing,29.9,1).editor_resynchronized);
  const auto rq=app.begin_mutation(AndroidEditorIntent::upload,30.0);
  assert(rq.valid);
  const auto e=lab_proof(30.02,2);
  auto deny=[&](const std::string& frame,const AndroidDraftEncodedRequest& expected){
    auto outcome=framing.mutation(frame,expected,e);
    assert(!outcome.reached_backend && !outcome.trusted_reply_provenance &&
           ingress.last_ingress_nonce()==1 && !backend.has_draft());
  };
  const auto good=framed_lab_request(rq.method,rq.path,rq.body);
  deny(framed_lab_request("POST",kBackendMissionUpdatePath,rq.body),rq);
  deny(framed_lab_request("GET",rq.path,""),rq);
  deny(framed_lab_request("POST",rq.path,"{}"),rq);
  auto wrong=rq;wrong.sequence++;
  deny(good,wrong);
  wrong=rq;wrong.path=kBackendMissionLoadPath;
  deny(good,wrong);
  wrong=rq;wrong.body+=" ";
  deny(good,wrong);
  wrong=rq;wrong.body.replace(wrong.body.find("\"sequence\":"),12,"\"sequence\":999");
  deny(framed_lab_request(rq.method,rq.path,wrong.body),wrong);
  // GET cannot carry a body or be correlated to the wrong draft path.
  const auto sync=app.begin_resync(30.1);
  assert(!sync.valid); // A mutation is pending. No concurrent GET/POST.
  assert(!framing.mutation(good,rq,lab_proof(30.02,2)).network_io_performed);
  assert(backend.has_draft() && !gateway.emits_hardware_commands());
}
