#include "backend_draft_ingress_lab_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  NativeOfflineDraftIngressLab ingress(wire);
  NativeOfflineAndroidDraftCoordinator app;
  app.show(sync_screen());
  auto rq=app.begin_resync(20.0);
  assert(rq.valid);
  auto success=ingress.snapshot(rq,lab_proof(20.02,1));
  assert(success.reached_backend && ingress.last_ingress_nonce()==1);
  auto dupe=ingress.snapshot(rq,lab_proof(20.03,1));
  assert(!dupe.reached_backend && !dupe.trusted_reply_provenance);
  auto changed_epoch=lab_proof(20.04,2);
  changed_epoch.independently_verified_session_epoch=4;
  assert(!ingress.snapshot(rq,changed_epoch).reached_backend);
  auto changed_principal=lab_proof(20.05,2,"OTHER_OPERATOR");
  assert(!ingress.snapshot(rq,changed_principal).reached_backend);
  assert(ingress.last_ingress_nonce()==1);
  assert(app.complete_resync(rq,
       draft_ingress_lab_sync_reply(rq,success,20.0,20.07),20.08)
       .editor_resynchronized);
  auto upload=app.begin_mutation(AndroidEditorIntent::upload,20.1);
  assert(upload.valid);
  auto wrong=upload;
  wrong.body+=" ,\"authenticated\":true";
  assert(ingress.mutation(wrong,lab_proof(20.12,2)).reached_backend);
  // A malformed mutation is still consumed by backend but must not modify it.
  assert(!backend.has_draft() && ingress.last_ingress_nonce()==2);
  assert(!ingress.mutation(upload,lab_proof(20.13,2)).reached_backend);
  auto actual=ingress.mutation(upload,lab_proof(20.14,3));
  assert(actual.reached_backend && actual.wire.status==200 && backend.has_draft());
  auto rollback=ingress.snapshot(rq,lab_proof(19.0,4));
  assert(!rollback.reached_backend && ingress.fault_latched());
  assert(!ingress.snapshot(rq,lab_proof(21.0,5)).reached_backend);
  assert(!gateway.has_active_mission() && !gateway.emits_hardware_commands());
}
