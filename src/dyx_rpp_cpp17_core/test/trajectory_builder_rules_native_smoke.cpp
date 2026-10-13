#include "dyx_rpp_cpp17_core/trajectory_builder.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace dyx::rpp;
int main() {
  std::vector<Point2D> p{{0,0},{0,8},{1.5,8},{1.5,0}};
  TrajectoryBuildConfig cfg; cfg.extension_enabled=true;
  cfg.skip_row_end_indices.insert(1);
  const auto skipped=NativeTrajectoryBuilder(cfg).build(p);
  assert(skipped.dummy_count==0);
  assert(skipped.transitions[1].reason==RowTransitionReason::skipped_by_operator);
  cfg.skip_row_end_indices.clear();
  cfg.extension_trigger_m=1.5; // boundary is STRICT less-than
  const auto boundary=NativeTrajectoryBuilder(cfg).build(p);
  assert(boundary.dummy_count==0);
  assert(boundary.transitions[1].reason==RowTransitionReason::gap_not_under_trigger);
  cfg.extension_trigger_m=3.0;
  cfg.extension_distance_m=2.5;
  const auto edited=NativeTrajectoryBuilder(cfg).build(p);
  assert(edited.dummy_count==1);
  assert(std::abs(edited.transitions[1].dummy.y-10.5)<1e-12);
  cfg.extension_enabled=false;
  const auto disabled=NativeTrajectoryBuilder(cfg).build(p);
  assert(disabled.dummy_count==0);
  assert(disabled.transitions[1].reason==RowTransitionReason::disabled);
  // A point set with no reverse row direction is not a row end.
  cfg.extension_enabled=true;
  const auto ordinary=NativeTrajectoryBuilder(cfg).build({{0,0},{3,0},{3,1},{5,1}});
  assert(ordinary.dummy_count==0);
  const auto again=NativeTrajectoryBuilder(cfg).build(p);
  const auto again2=NativeTrajectoryBuilder(cfg).build(p);
  assert(again.navigation.size()==again2.navigation.size());
  for (std::size_t i=0;i<again.navigation.size();++i) {
    assert(again.navigation[i].point.x==again2.navigation[i].point.x);
    assert(again.navigation[i].point.y==again2.navigation[i].point.y);
    assert(again.navigation[i].point_type==again2.navigation[i].point_type);
    assert(again.navigation[i].marking_index==again2.navigation[i].marking_index);
  }
  std::cout << "trajectory_builder_rules_native_smoke passed\n";
}
