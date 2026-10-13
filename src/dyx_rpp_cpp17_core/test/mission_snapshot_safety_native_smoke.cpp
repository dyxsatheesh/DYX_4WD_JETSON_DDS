#include "dyx_rpp_cpp17_core/mission_snapshot.hpp"
#include <cassert>
#include <limits>
using namespace dyx::rpp;
int main() {
  const std::vector<Point2D> nav{{0,0},{1,0}};
  const std::vector<Point2D> marks{{0,0},{1,0}};
  const std::vector<int> types{2,2}, ids{0,1};
  PreparedPathSnapshot s{nav,marks,types,ids,{}};
  const auto signature=compute_path_signature(s);
  PendingPreparedSnapshot p;
  p.set_ready(true);p.set_markings(marks);p.set_point_types(types);
  p.set_marking_indices(ids);p.set_navigation(nav);p.set_signature("bad");
  assert(p.try_install().status==SnapshotInstallStatus::invalid_signature);
  p.set_signature(signature);
  p.set_navigation({{0,0},{2,0}});
  assert(p.try_install().status==SnapshotInstallStatus::invalid_content);
  p.set_navigation({{0,0},{1,0}});
  assert(p.try_install().committed());
  PendingPreparedSnapshot q;
  q.set_ready(true);q.set_markings(marks);q.set_point_types(types);
  q.set_marking_indices(ids);q.set_navigation(nav);
  auto different=s; different.navigation_points[1].x=1.5;
  // Compare when content is individually coherent but the signature is stale.
  different.marking_points[1].x=1.5;
  q.set_navigation(different.navigation_points);
  q.set_markings(different.marking_points);
  q.set_signature(signature);
  assert(q.try_install().status==SnapshotInstallStatus::signature_mismatch);
  q.set_signature(compute_path_signature(different));
  assert(q.try_install().committed());
  q.clear_markings();q.set_ready(true);
  assert(q.try_install().status==SnapshotInstallStatus::incomplete);
  s.navigation_points[0].x=std::numeric_limits<double>::quiet_NaN();
  bool threw=false;
  try { compute_path_signature(s); } catch(const std::invalid_argument&) {threw=true;}
  assert(threw);
}
