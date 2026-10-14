#include "backend_lifecycle_fixture.hpp"
int main() {
  NativeOfflineRequestLifecycle lifecycle;
  const auto read = lifecycle.begin(offline_get(1, kBackendMissionStatusPath), 1.0);
  assert(read.allowed() && read.ticket.valid());
  assert(!read.physical_command_emitted && lifecycle.active_count()==1);
  const auto live = lifecycle.begin(offline_get(2, kBackendLiveStatusPath, 1.1), 1.1);
  assert(live.allowed() && live.kind == OfflineRequestKind::telemetry);
  const auto command = lifecycle.begin(offline_post(3, kBackendMissionPausePath, 1.2), 1.2);
  assert(command.allowed() && command.kind == OfflineRequestKind::command);
  assert(lifecycle.complete(read.ticket,1.3,128).code==OfflineLifecycleCode::completed);
  assert(lifecycle.complete(live.ticket,1.3,256).code==OfflineLifecycleCode::completed);
  assert(lifecycle.complete(command.ticket,1.3,512).code==OfflineLifecycleCode::completed);
  assert(!lifecycle.complete(command.ticket,1.4,10).allowed());
  assert(lifecycle.active_count()==0);
  assert(!lifecycle.begin(offline_get(2,kBackendMissionStatusPath,1.4),1.4).allowed());
  // No I/O, privilege, vehicle command, or physical outputs in this API.
}
