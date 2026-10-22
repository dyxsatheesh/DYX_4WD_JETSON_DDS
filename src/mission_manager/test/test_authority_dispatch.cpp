#include "dyx_mission_manager/native_authority_dispatch.hpp"
#include <stdexcept>
#include <iostream>
using namespace dyx::http_backend;
using namespace dyx::rpp;
static void ok(bool result, const char* reason) {
  if (!result) throw std::runtime_error(reason);
}
static BackendMissionApiRequest cmd(const char* path) {
  BackendMissionApiRequest r{};
  r.path=path; r.method="POST"; r.mission_run_id="r1";
  r.principal.authenticated=true;
  r.principal.operator_session_id="s1";
  r.principal.owns_control_lease=false;
  return r;
}
static AuthoritySafetySnapshot full_safety() {
  AuthoritySafetySnapshot s{};
  s.authority_epoch=1; s.collected_monotonic_sec=1.0;
  s.trusted_adapter_bundle=true;
  auto& e=s.facts;
  e.mission_loaded=true; e.stop_latched=false; e.telemetry_fault=false;
  e.rtk_fixed=true; e.receiver_age_trusted=true; e.position_fresh=true;
  e.rear_reference_calibrated=true; e.battery_reserve_verified=true;
  e.obstacle_clear_verified=true; e.physical_estop_clear_verified=true;
  e.operator_lease_verified=true; e.px4_command_path_commissioned=true;
  e.spray_safety_commissioned=true; e.timing_fault=false;
  return s;
}
int main() {
  NativeMotionAuthority a{};
  auto r=cmd(kBackendMissionStartPath);
  auto d=assess_authority_dispatch(a,r,MissionStage::ready,false,1.1);
  ok(!d.authority.software_eligible,"no lease must deny");
  ok(!d.transition.dispatch_authorized,"no lease dispatched");
  MotionLeaseSnapshot lease{};
  lease.session_id="s1";lease.run_id="r1";lease.epoch=1;
  lease.issued_monotonic_sec=0.0;lease.expires_monotonic_sec=10.0;
  lease.server_verified=true;
  a.observe_lease(lease);
  a.observe_safety(full_safety());
  d=assess_authority_dispatch(a,r,MissionStage::ready,false,1.1);
  ok(d.authority.software_eligible,"full trusted bench fixture eligible");
  ok(!d.authority.motion_authorized,"fixture granted motion");
  ok(!d.transition.dispatch_authorized,"fixture dispatched mission");
  d=assess_authority_dispatch(a,r,MissionStage::ready,true,1.1);
  ok(!d.authority.software_eligible,"mailbox STOP ignored");
  ok(!d.transition.dispatch_authorized,"STOP allowed dispatch");
  a.stop();
  d=assess_authority_dispatch(a,r,MissionStage::ready,false,1.1);
  ok(!d.authority.software_eligible,"latched STOP ignored");
  ok(!d.transition.dispatch_authorized,"latched STOP dispatch");
  std::cout << "PASS: 9 authority-to-ROS dispatch assertions; no motion enabled\n";
}
