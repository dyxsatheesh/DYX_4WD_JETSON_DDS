#include "dyx_rpp_cpp17_core/trajectory_builder.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace dyx::rpp;

int main() {
  const std::vector<Point2D> two{{0.0,0.0},{0.0,1.0}};
  NativeTrajectoryBuilder off;
  auto a=off.build(two);
  assert(a.dummy_count == 0 && a.navigation.size()==21);
  assert(a.navigation.front().point_type==point_marking);
  assert(a.navigation.back().point_type==point_marking);
  assert(a.marking_raw_indices.size()==2);
  assert(a.transitions.front().reason==RowTransitionReason::last_pair);
  assert(a.navigation[a.marking_raw_indices[1]].marking_index==1);
  assert(a.navigation[a.marking_raw_indices[1]].point.y==1.0);
  // Two-point missions must never invent a row-extension dummy, even when ON.
  TrajectoryBuildConfig cfg; cfg.extension_enabled=true;
  auto b=NativeTrajectoryBuilder(cfg).build(two);
  assert(b.dummy_count==0 && b.navigation.size()==a.navigation.size());
  // Forward row, sideways transfer, reverse-direction next row.
  const std::vector<Point2D> serpentine{{0,0},{0,8},{1.5,8},{1.5,0}};
  const auto p=NativeTrajectoryBuilder(cfg).build(serpentine);
  assert(p.dummy_count == 1);
  assert(p.transitions.size()==3);
  assert(p.transitions[0].reason==RowTransitionReason::gap_not_under_trigger);
  assert(p.transitions[1].reason==RowTransitionReason::added);
  assert(p.transitions[2].reason==RowTransitionReason::last_pair);
  assert(p.transitions[1].has_dummy);
  assert(p.transitions[1].dummy.x==0.0 && p.transitions[1].dummy.y==12.0);
  assert(std::abs(p.transitions[1].transfer_angle_deg-90)<1e-9);
  assert(std::abs(p.transitions[1].reversal_angle_deg-180)<1e-9);
  int dummy_count=0;
  for (const auto &point:p.navigation) if (point.point_type==point_dummy_alignment) ++dummy_count;
  assert(dummy_count == 1);
  for (std::size_t i=0;i<serpentine.size();++i) {
    const auto &m=p.navigation.at(p.marking_raw_indices.at(i));
    assert(m.point.x==serpentine[i].x && m.point.y==serpentine[i].y);
    assert(m.point_type==point_marking && m.marking_index==static_cast<int>(i));
  }
  // Already migrated native RPP path geometry must accept all semantic metadata.
  const auto indexed=PathGeometryIndex::build(p.navigation);
  assert(indexed.semantic_anchors.size()==serpentine.size()+p.dummy_count);
  assert(indexed.total_length>16.0);
  std::cout << "trajectory_builder_native_smoke passed\n";
}
