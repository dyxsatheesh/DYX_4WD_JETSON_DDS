#include "backend_ingress_fixture.hpp"
int main() {
  NativeOfflineControlSessions s;
  auto a=operator_a();
  auto forged=a;forged.authentication_verified=false;
  assert(s.acquire(forged,3.0).code==IngressDecisionCode::authentication_required);
  auto invalid=a;invalid.session_id="x";
  assert(s.acquire(invalid,3.0).code==IngressDecisionCode::invalid_identity);
  auto no_scope=a;no_scope.control_scope_verified=false;
  assert(s.acquire(no_scope,3.0).code==IngressDecisionCode::control_scope_required);
  auto lease=s.acquire(a,3.0);
  assert(lease.allowed());
  assert(s.authorize(a,IngressOperation::control,0,1,3.1).code==
         IngressDecisionCode::generation_mismatch);
  assert(s.authorize(a,IngressOperation::control,lease.generation,0,3.2).code==
         IngressDecisionCode::invalid_sequence);
  assert(s.authorize(a,IngressOperation::control,lease.generation,1,3.3).allowed());
  s.revoke();
  assert(s.authorize(a,IngressOperation::control,lease.generation,2,3.4).code==
         IngressDecisionCode::lease_required);
  auto new_lease=s.acquire(a,3.5);
  assert(new_lease.allowed() && new_lease.generation!=lease.generation);
  assert(s.authorize(a,IngressOperation::control,lease.generation,3,3.6).code==
         IngressDecisionCode::generation_mismatch);
  assert(s.authorize(a,IngressOperation::control,new_lease.generation,1,3.7).allowed());
  assert(s.authorize(no_scope,IngressOperation::stop,0,7,3.8).code==
         IngressDecisionCode::control_scope_required);
  assert(s.authorize(a,IngressOperation::stop,0,7,3.9).allowed());
}
