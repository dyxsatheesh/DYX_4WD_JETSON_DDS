#include "backend_ingress_fixture.hpp"
int main() {
  NativeOfflineControlSessions s;
  auto a=operator_a();
  assert(!s.authorize(a,IngressOperation::control,1,1,1.0).allowed());
  const auto lease=s.acquire(a,1.0);
  assert(lease.allowed() && lease.generation>0 && lease.principal.owns_control_lease);
  assert(!lease.hardware_command_emitted);
  assert(s.acquire(a,1.1).code==IngressDecisionCode::lease_busy);
  assert(s.acquire(operator_b(),1.2).code==IngressDecisionCode::lease_busy);
  assert(s.authorize(viewer(),IngressOperation::read,0,0,1.3).allowed());
  const auto request=s.authorize(a,IngressOperation::control,lease.generation,3,1.4);
  assert(request.allowed() && request.principal.operator_session_id==a.session_id);
  assert(s.authorize(a,IngressOperation::control,lease.generation,3,1.5).code==
         IngressDecisionCode::sequence_replay);
  assert(s.authorize(a,IngressOperation::control,lease.generation,2,1.6).code==
         IngressDecisionCode::sequence_replay);
  assert(s.authorize(a,IngressOperation::control,lease.generation,4,1.7).allowed());
  assert(s.authorize(operator_b(),IngressOperation::control,lease.generation,5,1.8).code==
         IngressDecisionCode::generation_mismatch);
  assert(s.renew(a,lease.generation,2.0).allowed());
}
