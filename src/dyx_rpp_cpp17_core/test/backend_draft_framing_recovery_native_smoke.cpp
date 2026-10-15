#include "backend_draft_http_framing_fixture.hpp"
int main() {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionUpload backend(gateway);
  NativeBackendUploadHttpWire wire(backend);
  NativeOfflineDraftIngressLab ingress(wire);
  NativeOfflineDraftHttpFramingLab framing(ingress);
  NativeOfflineAndroidDraftCoordinator app;
  app.show(sync_screen());
  assert(framed_sync(app,framing,40.0,1).editor_resynchronized);
  const auto request=app.begin_mutation(AndroidEditorIntent::upload,40.1);
  assert(request.valid);
  const auto raw=framed_lab_request(request.method,request.path,request.body);
  const auto accepted=framing.mutation(raw,request,lab_proof(40.12,2));
  assert(accepted.reached_backend && accepted.wire.status==200);
  assert(!app.expire_mutation(42.2).verified_backend_decision);
  assert(app.requires_resync());
  // A retry cannot assume the absent response means the mutation failed.
  assert(!framing.mutation(raw,request,lab_proof(42.21,2)).reached_backend);
  const auto replay=framing.mutation(raw,request,lab_proof(42.22,3));
  assert(replay.reached_backend && replay.wire.status==409);
  assert(!app.begin_mutation(AndroidEditorIntent::upload,42.23).valid);
  const auto restored=framed_sync(app,framing,42.3,4);
  assert(restored.editor_resynchronized && restored.draft.revision==1);
  assert(!app.requires_resync());
  assert(!gateway.has_active_mission() && !gateway.emits_hardware_commands());
}
