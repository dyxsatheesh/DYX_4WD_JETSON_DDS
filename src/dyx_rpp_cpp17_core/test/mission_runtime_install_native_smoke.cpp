#include "dyx_rpp_cpp17_core/mission_runtime_install.hpp"
#include <cassert>
#include <iostream>
#include <string>
#include <vector>
using namespace dyx::rpp;

static PreparedPathSnapshot example() {
  const std::vector<RawPathPoint> nav{
    {{0,0}, point_marking, 0},
    {{1,0}, point_pass_through,-1},
    {{2,0}, point_marking, 1},
    {{2.5,0}, point_dummy_alignment,-1},
    {{3.0,0}, point_pass_through,-1},
    {{4.0,0}, point_marking,2}};
  return signed_snapshot(nav, {{0,0}, {2,0}, {4,0}});
}
static void stage(PendingPreparedSnapshot &pending, const PreparedPathSnapshot &s) {
  pending.set_marking_indices(s.marking_indices);
  pending.set_navigation(s.navigation_points);
  pending.set_signature(s.signature);
  pending.set_point_types(s.point_types);
  pending.set_markings(s.marking_points);
  pending.set_ready(true);
}
int main() {
  auto s=example();
  PendingPreparedSnapshot pending;
  NativeMissionInstaller installer;
  stage(pending,s);
  const std::string run="RUN_20261013_X1";
  auto denied=installer.install_from_pending(pending,run,MissionMode::automatic,false,true);
  assert(denied.status==NativeInstallStatus::unauthorized);
  assert(installer.current()==nullptr);
  denied=installer.install_from_pending(pending,run,MissionMode::automatic,true,false);
  assert(denied.status==NativeInstallStatus::vehicle_not_disarmed);
  auto done=installer.install_from_pending(pending,run,MissionMode::automatic,true,true);
  assert(done.installed());
  assert(installer.current()!=nullptr);
  const auto &active=*installer.current();
  assert(active.mission_run_id==run);
  assert(active.sequencer.stage()==MissionStage::ready);
  assert(active.marking_goals.size()==3);
  assert(active.marking_goals[0].raw_path_index==0);
  assert(active.marking_goals[1].raw_path_index==2);
  assert(active.marking_goals[2].raw_path_index==5);
  assert(active.marking_goals[2].goal_label=="P0003");
  const auto p3_hash=native_goal_instance_id(run,s.signature,5,3);
  assert(active.marking_goals[2].terminal_identity.find("|INSTANCE:"+p3_hash)!=std::string::npos);
  assert(active.marking_goals[2].terminal_identity.find("|GOAL:P0003|")!=std::string::npos);
  assert(active.snapshot.signature==s.signature);
  assert(active.geometry.segments.size()>0);
  // Pending duplicate is never silently installed into an active runtime.
  assert(installer.install_from_pending(pending,"RUN_20261013_Y2",MissionMode::manual,true,true).status==NativeInstallStatus::already_active);
  assert(installer.retire(true,true).status==NativeInstallStatus::not_stopped);
  assert(installer.stop_sequence(false).status==NativeInstallStatus::unauthorized);
  assert(installer.stop_sequence(true).installed());
  assert(installer.retire(true,false).status==NativeInstallStatus::vehicle_not_disarmed);
  assert(installer.retire(true,true).installed());
  assert(installer.current()==nullptr);
  // A run ID cannot be replayed even after a mission has been retired.
  pending.clear_all();stage(pending,s);
  assert(installer.install_from_pending(pending,run,MissionMode::automatic,true,true).status==NativeInstallStatus::run_id_reused);
  assert(installer.install_from_pending(pending,"RUN_20261013_Z3",MissionMode::manual,true,true).installed());
  assert(installer.current()->sequencer.stage()==MissionStage::ready);
  std::cout<<"mission runtime install smoke passed\n";
}
