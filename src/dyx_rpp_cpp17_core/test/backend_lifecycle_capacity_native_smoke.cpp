#include "backend_lifecycle_fixture.hpp"
int main() {
  OfflineRequestLifecycleConfig cfg;cfg.max_inflight=1;
  NativeOfflineRequestLifecycle lifecycle(cfg);
  auto first=lifecycle.begin(offline_get(1,kBackendMissionStatusPath,1.0),1.0);
  assert(first.allowed());
  auto busy=lifecycle.begin(offline_get(2,kBackendLiveStatusPath,1.1),1.1);
  assert(busy.code==OfflineLifecycleCode::busy);
  // Verified STOP must have its own independent ingress path, not this queue.
  auto stop=lifecycle.begin(offline_post(3,kBackendMissionStopPath,1.2),1.2);
  assert(stop.code==OfflineLifecycleCode::stop_out_of_band);
  assert(!stop.ticket.valid() && !stop.physical_command_emitted);
  assert(lifecycle.active_count()==1);
  assert(lifecycle.begin(offline_get(1,kBackendMissionStatusPath,1.3),1.3).code==
         OfflineLifecycleCode::request_id_replayed);
  lifecycle.fence_epoch();
  assert(lifecycle.active_count()==0);
  assert(lifecycle.complete(first.ticket,1.4,20).code==OfflineLifecycleCode::invalid_ticket);
  assert(!lifecycle.cancel(first.ticket));
  const auto later=lifecycle.begin(offline_get(4,kBackendLiveStatusPath,1.5),1.5);
  assert(later.allowed() && later.ticket.epoch!=first.ticket.epoch);
  assert(lifecycle.cancel(later.ticket));
  assert(!lifecycle.cancel(later.ticket));
  assert(lifecycle.active_count()==0);
  // Even a latched clock fault does not gate authenticated STOP externally.
  assert(lifecycle.begin(offline_get(5,kBackendLiveStatusPath,0.1),0.1).code ==
         OfflineLifecycleCode::host_clock_fault);
  assert(lifecycle.clock_fault_latched());
  auto repeated_stop = lifecycle.begin(offline_post(3,kBackendMissionStopPath,0.0),0.0);
  assert(repeated_stop.code == OfflineLifecycleCode::stop_out_of_band);
  assert(!repeated_stop.physical_command_emitted);
}
