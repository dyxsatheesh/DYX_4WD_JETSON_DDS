#include "dyx_rpp_cpp17_core/trajectory_builder.hpp"
#include <cassert>
#include <vector>
int main() {
  using namespace dyx::rpp;
  const std::vector<Point2D> points{{0,0},{0,1},{1,1},{1,0}};
  TrajectoryBuildConfig cfg{};
  cfg.extension_enabled = true;
  cfg.extension_trigger_m = 3.0;
  const auto normal = NativeTrajectoryBuilder(cfg).build(points);
  assert(normal.transitions.at(0).reason == RowTransitionReason::added);
  assert(normal.dummy_count > 0);
  cfg.skip_row_end_indices.insert(0);
  const auto skipped = NativeTrajectoryBuilder(cfg).build(points);
  assert(skipped.transitions.at(0).reason == RowTransitionReason::skipped_by_operator);
  assert(skipped.dummy_count + 1 == normal.dummy_count);
  assert(skipped.marking_raw_indices.size() == points.size());
  for (std::size_t i=0; i<points.size(); ++i) {
    const auto &p=skipped.navigation.at(skipped.marking_raw_indices.at(i)).point;
    assert(p.x == points[i].x && p.y == points[i].y);
  }
  return 0;
}
