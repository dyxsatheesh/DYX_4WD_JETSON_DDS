#include "dyx_mission_manager/native_motion_authority.hpp"
#include <cstdlib>
#include <iostream>
#include <stdexcept>
using namespace dyx::http_backend;
using namespace dyx::rpp;
static void verify(bool ok, const char* why) {if (!ok) throw std::runtime_error(why);}
static BackendMissionApiRequest command(const char* path,
                                        const char* session="op1",
                                        const char* run="run1") {
  BackendMissionApiRequest r{};
  r.method="POST";r.path=path;r.mission_run_id=run;
  r.principal.authenticated=true;
  r.principal.operator_session_id=session;
  // No caller permission boolean is trusted by NativeMotionAuthority.
  r.principal.owns_control_lease=false;
  return r;
}
static AuthoritySafetySnapshot full_evidence(double at, std::uint64_t epoch) {
  AuthoritySafetySnapshot x{};
  x.trusted_adapter_bundle=true;x.collected_monotonic_sec=at;
  x.authority_epoch=epoch;
  auto& e=x.facts;
  e.mission_loaded=true;e.stop_latched=false;e.telemetry_fault=false;
  e.rtk_fixed=true;e.receiver_age_trusted=true;e.position_fresh=true;
  e.rear_reference_calibrated=true;e.battery_reserve_verified=true;
  e.obstacle_clear_verified=true;e.physical_estop_clear_verified=true;
  e.operator_lease_verified=true;e.px4_command_path_commissioned=true;
  e.spray_safety_commissioned=true;e.timing_fault=false;
  return x;
}
int main() {
  int count=0;
  auto check=[&](bool v,const char* msg){verify(v,msg);++count;};
  NativeMotionAuthority a;
  auto start=command(kBackendMissionStartPath);
  auto d=a.assess(start,MissionStage::ready,10.0);
  check(!d.software_eligible,"no lease must deny");
  MotionLeaseSnapshot lease{};
  lease.session_id="op1";lease.run_id="run1";lease.epoch=1;
  lease.issued_monotonic_sec=9.0;lease.expires_monotonic_sec=20.0;
  lease.server_verified=true;
  a.observe_lease(lease);
  a.observe_safety(full_evidence(10.0,1));
  d=a.assess(start,MissionStage::ready,10.1);
  check(d.software_eligible,"fresh start eligible");
  check(!d.motion_authorized,"no software authority may actuate");
  check(!a.assess(command(kBackendMissionStartPath,"other"),MissionStage::ready,10.1).software_eligible,"other session denied");
  check(!a.assess(command(kBackendMissionStartPath,"op1","other"),MissionStage::ready,10.1).software_eligible,"other run denied");
  check(!a.assess(start,MissionStage::paused,10.1).software_eligible,"wrong stage denied");
  check(!a.assess(start,MissionStage::ready,10.26).software_eligible,"stale evidence denied");
  auto unsafe=full_evidence(10.3,1);unsafe.facts.physical_estop_clear_verified=false;
  a.observe_safety(unsafe);
  check(!a.assess(start,MissionStage::ready,10.31).software_eligible,"estop denied");
  a.observe_safety(full_evidence(10.4,1));
  check(a.assess(command(kBackendMissionResumePath),MissionStage::paused,10.41).software_eligible,"resume stage eligible");
  check(a.assess(command(kBackendMissionNextPath),MissionStage::await_next,10.41).software_eligible,"next stage eligible");
  check(a.assess(command(kBackendMissionSkipPath),MissionStage::approaching,10.41).software_eligible,"skip stage eligible");
  check(!a.assess(start,MissionStage::ready,20.0).software_eligible,"expired lease denied");
  lease.epoch=2;lease.issued_monotonic_sec=20.0;lease.expires_monotonic_sec=40.0;
  a.observe_lease(lease);a.observe_safety(full_evidence(20.1,2));
  check(a.assess(start,MissionStage::ready,20.2).software_eligible,"renewed epoch allowed");
  a.observe_lease(lease);
  check(!a.assess(start,MissionStage::ready,20.2).software_eligible,"replayed epoch denied");
  lease.epoch=3;lease.issued_monotonic_sec=21.0;lease.expires_monotonic_sec=30.0;
  a.observe_lease(lease);a.observe_safety(full_evidence(21.1,3));
  a.stop();
  check(a.stop_latched(),"stop latched");
  check(!a.assess(start,MissionStage::ready,21.2).software_eligible,"stop denies");
  lease.epoch=4;a.observe_lease(lease);a.observe_safety(full_evidence(21.2,4));
  check(!a.assess(start,MissionStage::ready,21.3).software_eligible,"stop cannot be cleared by new evidence");
  std::cout<<"PASS: "<<count<<" native motion authority checks; physical authorization always false\n";
}
