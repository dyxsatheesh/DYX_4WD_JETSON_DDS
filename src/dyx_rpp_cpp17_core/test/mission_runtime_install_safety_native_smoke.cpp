#include "dyx_rpp_cpp17_core/mission_runtime_install.hpp"
#include <cassert>
#include <iostream>
#include <string>
#include <vector>
using namespace dyx::rpp;

static PreparedPathSnapshot sample() {
  return signed_snapshot({{{0,0},point_marking,0},{{1,0},point_marking,1}},
                        {{0,0},{1,0}});
}
static void stage(PendingPreparedSnapshot &p,const PreparedPathSnapshot &s) {
  p.set_navigation(s.navigation_points);
  p.set_markings(s.marking_points);
  p.set_point_types(s.point_types);
  p.set_marking_indices(s.marking_indices);
  p.set_signature(s.signature);
  p.set_ready(true);
}
int main() {
  assert(!valid_native_mission_run_id("short"));
  assert(!valid_native_mission_run_id("RUN|INJECT:X"));
  assert(!valid_native_mission_run_id("RUN SPACE 01"));
  assert(valid_native_mission_run_id("RID-20261013-01"));
  PendingPreparedSnapshot p;
  NativeMissionInstaller runtime;
  const auto s=sample();
  assert(runtime.install_from_pending(p,"RID-20261013-01",MissionMode::automatic,true,true).status==NativeInstallStatus::snapshot_not_ready);
  stage(p,s);
  assert(runtime.install_from_pending(p,"bad|run-id",MissionMode::automatic,true,true).status==NativeInstallStatus::invalid_run_id);
  p.set_signature(std::string(64,'0'));
  assert(runtime.install_from_pending(p,"RID-20261013-01",MissionMode::automatic,true,true).status==NativeInstallStatus::snapshot_invalid);
  assert(runtime.current()==nullptr);
  p.set_signature(s.signature);
  p.set_navigation({{0,0},{8,0}});  // signature mismatch
  assert(runtime.install_from_pending(p,"RID-20261013-01",MissionMode::automatic,true,true).status==NativeInstallStatus::snapshot_invalid);
  p.set_navigation(s.navigation_points);
  assert(runtime.install_from_pending(p,"RID-20261013-01",MissionMode::automatic,true,true).installed());
  assert(runtime.current()->sequencer.stage()==MissionStage::ready);
  assert(runtime.retire(false,true).status==NativeInstallStatus::unauthorized);
  assert(runtime.retire(true,true).status==NativeInstallStatus::not_stopped);
  assert(runtime.stop_sequence(true).installed());
  assert(runtime.retire(true,true).installed());
  assert(runtime.current()==nullptr);
  p.clear_all();stage(p,s);
  assert(runtime.install_from_pending(p,"RID-20261013-01",MissionMode::automatic,true,true).status==NativeInstallStatus::run_id_reused);
  assert(runtime.install_from_pending(p,"RID-20261013-02",MissionMode::automatic,true,true).installed());
  assert(runtime.current()->marking_goals[1].terminal_identity!=
         std::string("RUN:RID-20261013-01|PATH:")+s.signature+"|RAW:1|GOAL:P0002|INSTANCE:"+
         native_goal_instance_id("RID-20261013-01",s.signature,1,1));
  std::cout<<"mission runtime install safety passed\n";
}
