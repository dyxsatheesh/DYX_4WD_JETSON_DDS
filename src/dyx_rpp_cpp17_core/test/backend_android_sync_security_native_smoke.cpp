#include "backend_android_sync_fixture.hpp"
#include <limits>
int main() {
  NativeOfflineAndroidMissionEditor editor;
  editor.show(sync_screen());
  NativeOfflineAndroidDraftSync sync;
  assert(!sync.begin("evil?mission=other",9.0).valid);
  auto request=sync.begin("RUN_H4R_0001",10.0);
  assert(request.valid && sync.pending());
  assert(!sync.begin("RUN_H4R_0001",10.01).valid);
  BackendHttpWireResponse no_draft{404,"{\"ok\":false,\"code\":\"NO_PENDING_DRAFT\"}"};
  auto trusted=sync_verified(request,no_draft,10.0,10.05);
  trusted.verified_authenticated_transport=false;
  assert(!sync.complete(request,trusted,10.10,editor).verified);
  assert(!sync.pending());
  assert(!sync.complete(request,sync_verified(request,no_draft,10.0,10.05),10.11,editor).verified);
  request=sync.begin("RUN_H4R_0001",10.20);
  trusted=sync_verified(request,no_draft,10.20,10.25);
  trusted.verified_request_path="/api/v1/mission/draft?mission_run_id=RUN_OTHER";
  assert(!sync.complete(request,trusted,10.30,editor).verified);
  request=sync.begin("RUN_H4R_0001",10.40);
  trusted=sync_verified(request,no_draft,10.40,10.45);
  ++trusted.verified_request_id;
  assert(!sync.complete(request,trusted,10.50,editor).verified);
  request=sync.begin("RUN_H4R_0001",10.60);
  trusted=sync_verified(request,no_draft,10.60,10.65);
  auto tampered=request;tampered.path="/api/v1/mission/draft?mission_run_id=WRONG";
  assert(!sync.complete(tampered,trusted,10.70,editor).verified);
  request=sync.begin("RUN_H4R_0001",10.80);
  trusted=sync_verified(request,no_draft,10.80,10.85);
  trusted.wire.physical_command_emitted=true;
  assert(!sync.complete(request,trusted,10.90,editor).verified);
  request=sync.begin("RUN_H4R_0001",11.0);
  trusted=sync_verified(request,no_draft,11.0,11.05);
  trusted.wire.network_io_performed=true;
  assert(!sync.complete(request,trusted,11.10,editor).verified);
  request=sync.begin("RUN_H4R_0001",11.2);
  trusted=sync_verified(request,no_draft,11.2,11.25);
  trusted.wire.content_type="text/plain";
  assert(!sync.complete(request,trusted,11.3,editor).verified);
  request=sync.begin("RUN_H4R_0001",11.4);
  trusted=sync_verified(request,no_draft,11.4,11.45);
  trusted.wire.body.append(5000,'x');
  assert(!sync.complete(request,trusted,11.5,editor).verified);
  assert(!sync.begin("bad/id",11.6).valid);
  assert(sync.last_request_id()==8);
}
