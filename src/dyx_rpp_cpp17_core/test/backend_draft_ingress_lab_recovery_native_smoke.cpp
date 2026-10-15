#include "backend_draft_ingress_lab_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  NativeOfflineDraftIngressLab ingress(wire);
  NativeOfflineAndroidDraftCoordinator app;
  app.show(sync_screen());
  assert(lab_sync(app,ingress,30.0,1).editor_resynchronized);
  auto upload=app.begin_mutation(AndroidEditorIntent::upload,30.1);
  assert(upload.valid);
  auto accepted=ingress.mutation(upload,lab_proof(30.12,2));
  assert(accepted.reached_backend && accepted.wire.status==200);
  // Deliberately lose the reply: Android cannot infer whether UPLOAD happened.
  assert(!app.expire_mutation(32.2).verified_backend_decision);
  assert(app.requires_resync());
  assert(!ingress.mutation(upload,lab_proof(32.21,2)).reached_backend);
  // A replay with a new transport nonce still hits the server's durable
  // command-sequence fence; it cannot create a second or modified draft.
  auto replay=ingress.mutation(upload,lab_proof(32.22,3));
  assert(replay.reached_backend && replay.wire.status==409);
  assert(backend.has_draft() && !backend.loaded());
  assert(!app.begin_mutation(AndroidEditorIntent::upload,32.24).valid);
  auto restored=lab_sync(app,ingress,32.3,4);
  assert(restored.editor_resynchronized && restored.draft_present &&
         restored.draft.revision==1);
  assert(!app.requires_resync());
  assert(!gateway.has_active_mission() && !gateway.emits_hardware_commands());
}
