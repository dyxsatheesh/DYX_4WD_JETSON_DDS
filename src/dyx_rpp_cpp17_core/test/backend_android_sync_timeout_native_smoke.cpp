#include "backend_android_sync_fixture.hpp"
#include <limits>
int main() {
  NativeOfflineAndroidMissionEditor editor;
  editor.show(sync_screen());
  NativeOfflineAndroidDraftSync sync;
  BackendHttpWireResponse empty{404,"{\"ok\":false,\"code\":\"NO_PENDING_DRAFT\"}"};
  auto request=sync.begin("RUN_H4R_0001",1.0);
  auto verified=sync_verified(request,empty,1.0,1.01);
  assert(!sync.complete(request,verified,4.0,editor).verified);
  request=sync.begin("RUN_H4R_0001",4.1);
  verified=sync_verified(request,empty,4.1,4.11);
  assert(!sync.complete(request,verified,7.0,editor).verified);
  request=sync.begin("RUN_H4R_0001",7.1);
  verified=sync_verified(request,empty,7.1,7.11);
  verified.verified_received_sec=7.25;
  assert(!sync.complete(request,verified,7.2,editor).verified);
  request=sync.begin("RUN_H4R_0001",7.3);
  verified=sync_verified(request,empty,7.29,7.35);
  assert(!sync.complete(request,verified,7.4,editor).verified);
  request=sync.begin("RUN_H4R_0001",7.5);
  verified=sync_verified(request,empty,7.5,7.55);
  verified.wire.body="{\"ok\":false,\"code\":\"OTHER_ROUTE_NOT_FOUND\"}";
  assert(!sync.complete(request,verified,7.6,editor).verified);
  request=sync.begin("RUN_H4R_0001",7.7);
  verified=sync_verified(request,empty,7.7,7.75);
  verified.wire.body="{\"ok\":false,\"code\":\"NO_PENDING_DRAFT\",\"admin\":true}";
  assert(!sync.complete(request,verified,7.8,editor).verified);
  request=sync.begin("RUN_H4R_0001",7.9);
  sync.invalidate();
  assert(!sync.complete(request,sync_verified(request,empty,7.9,7.92),7.95,editor).verified);
  assert(sync.last_request_id()==7);
  assert(!sync.begin("RUN_H4R_0001",7.0).valid); // rollback is latched
  assert(sync.clock_fault_latched());
  assert(!sync.begin("RUN_H4R_0001",8.5).valid);
}
