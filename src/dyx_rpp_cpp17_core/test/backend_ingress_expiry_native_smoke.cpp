#include "backend_ingress_fixture.hpp"
int main() {
  NativeOfflineControlSessions s({2.0});
  auto a=operator_a();
  const auto lease=s.acquire(a,10.0);
  assert(lease.allowed());
  assert(s.authorize(a,IngressOperation::control,lease.generation,1,11.99).allowed());
  assert(s.authorize(a,IngressOperation::control,lease.generation,2,12.0).code==
         IngressDecisionCode::lease_expired);
  assert(!s.has_active_lease());
  assert(s.acquire(a,12.1).allowed());
  const auto new_gen=s.generation();
  assert(new_gen!=lease.generation);
  assert(s.authorize(a,IngressOperation::control,new_gen,1,11.0).code==
         IngressDecisionCode::clock_fault);
  assert(s.fault_latched() && !s.has_active_lease());
  assert(s.acquire(a,13.0).code==IngressDecisionCode::clock_fault);
  // Authenticated STOP may be submitted for independent decision-level handling.
  assert(s.authorize(a,IngressOperation::stop,0,99,
     std::numeric_limits<double>::quiet_NaN()).allowed());
  assert(!s.reset_clock_fault_disarmed(true,false,1.0));
  assert(!s.reset_clock_fault_disarmed(false,true,1.0));
  assert(s.reset_clock_fault_disarmed(true,true,1.0));
  assert(!s.fault_latched());
  assert(s.acquire(a,1.1).allowed());
}
