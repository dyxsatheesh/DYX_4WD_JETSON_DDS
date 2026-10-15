#include "backend_draft_ingress_lab_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  NativeOfflineDraftIngressLab ingress(wire);
  NativeOfflineAndroidDraftCoordinator app;
  app.show(sync_screen());
  auto rq=app.begin_resync(10.0);
  assert(rq.valid);
  auto e=lab_proof(10.02,1);
  e.independently_verified_transport=false;
  assert(!ingress.snapshot(rq,e).reached_backend);
  e=lab_proof(10.02,1);
  e.independently_verified_principal.can_read_mission=false;
  assert(!ingress.snapshot(rq,e).reached_backend);
  auto good=ingress.snapshot(rq,lab_proof(10.02,1));
  assert(good.reached_backend);
  auto settled=app.complete_resync(rq,
       draft_ingress_lab_sync_reply(rq,good,10.0,10.04),10.05);
  assert(settled.editor_resynchronized);
  auto upload=app.begin_mutation(AndroidEditorIntent::upload,10.1);
  assert(upload.valid);
  e=lab_proof(10.12,2);
  e.independently_verified_principal.owns_control_lease=false;
  assert(!ingress.mutation(upload,e).reached_backend);
  e=lab_proof(10.12,2); e.independently_verified_px4_source=false;
  assert(!ingress.mutation(upload,e).reached_backend);
  e=lab_proof(10.12,2); e.px4_disarmed=false;
  assert(!ingress.mutation(upload,e).reached_backend);
  e=lab_proof(10.12,2); e.independently_observed_px4_sec=9.0;
  assert(!ingress.mutation(upload,e).reached_backend);
  e=lab_proof(10.12,2); e.independently_verified_auth_sec=7.0;
  assert(!ingress.mutation(upload,e).reached_backend);
  e=lab_proof(10.12,2);
  e.independently_verified_principal.operator_session_id="ANOTHER_OPERATOR";
  assert(!ingress.mutation(upload,e).reached_backend);
  assert(!backend.has_draft() && ingress.last_ingress_nonce()==1);
  const auto accepted=ingress.mutation(upload,lab_proof(10.12,2));
  assert(accepted.reached_backend && accepted.wire.status==200);
  assert(backend.has_draft() && !gateway.has_active_mission());
  auto same=app.complete_mutation(upload,
       draft_ingress_lab_mutation_reply(upload,accepted,10.1,10.14),10.15);
  assert(same.verified_backend_decision && app.requires_resync());
  assert(!gateway.emits_hardware_commands());
}
