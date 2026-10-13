#include "dyx_rpp_cpp17_core/mission_snapshot.hpp"
#include <cassert>
using namespace dyx::rpp;
int main() {
  const std::vector<Point2D> nav{{0,0},{0.05,0},{1,0}};
  const std::vector<Point2D> marks{{0,0},{1,0}};
  const std::vector<int> types{2,0,2}, ids{0,-1,1};
  PreparedPathSnapshot source{nav,marks,types,ids,{}};
  source.signature=compute_path_signature(source);
  PendingPreparedSnapshot p;
  assert(p.try_install().status==SnapshotInstallStatus::not_ready);
  p.set_ready(true); // Ready arrives BEFORE any data (DDS cross-topic order)
  assert(p.try_install().status==SnapshotInstallStatus::incomplete);
  p.set_signature(source.signature);
  p.set_markings(marks);
  p.set_marking_indices(ids);
  p.set_navigation(nav);
  assert(p.try_install().status==SnapshotInstallStatus::incomplete);
  p.set_point_types(types);
  auto installed=p.try_install();
  assert(installed.committed());
  assert(installed.snapshot->signature==source.signature);
  assert(p.try_install().status==SnapshotInstallStatus::not_ready);
  p.set_ready(true);
  assert(p.try_install().status==SnapshotInstallStatus::duplicate_install);
  p.invalidate_readiness();
  assert(p.try_install().status==SnapshotInstallStatus::not_ready);
  p.clear_all();
  p.set_navigation(nav);p.set_markings(marks);p.set_point_types(types);
  p.set_marking_indices(ids);p.set_signature(source.signature);p.set_ready(true);
  assert(p.try_install().committed()); // explicit new mission reset
}
