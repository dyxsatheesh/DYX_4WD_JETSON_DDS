#include "backend_lifecycle_fixture.hpp"
int main() {
  OfflineRequestLifecycleConfig cfg;
  cfg.max_processing_sec=0.5;
  cfg.max_arrival_age_sec=0.2;
  NativeOfflineRequestLifecycle lifecycle(cfg);
  auto stale=offline_get(1,kBackendMissionStatusPath,1.0);
  assert(lifecycle.begin(stale,1.21).code==OfflineLifecycleCode::stale_on_arrival);
  const auto accepted=lifecycle.begin(stale,1.1);
  // Backward host time must latch an integrity fault and fence the request.
  assert(accepted.code==OfflineLifecycleCode::host_clock_fault);
  assert(lifecycle.clock_fault_latched());
  assert(!lifecycle.reset_clock_fault_disarmed(true,false,1.3));
  assert(lifecycle.reset_clock_fault_disarmed(true,true,1.3));
  const auto next=lifecycle.begin(offline_get(2,kBackendMissionStatusPath,1.4),1.4);
  assert(next.allowed());
  assert(lifecycle.complete(next.ticket,2.0,5).code==OfflineLifecycleCode::timed_out);
  assert(lifecycle.active_count()==0);
  assert(lifecycle.complete(next.ticket,2.1,5).code==OfflineLifecycleCode::invalid_ticket);
  const auto current=lifecycle.begin(offline_get(3,kBackendLiveStatusPath,2.2),2.2);
  assert(current.allowed());
  assert(lifecycle.complete(current.ticket,2.3,65537).code==OfflineLifecycleCode::response_too_large);
  assert(lifecycle.active_count()==0);
}
