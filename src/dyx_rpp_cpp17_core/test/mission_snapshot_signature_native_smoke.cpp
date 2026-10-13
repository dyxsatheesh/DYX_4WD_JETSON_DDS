#include "dyx_rpp_cpp17_core/mission_snapshot.hpp"
#include <cassert>
#include <string>
using namespace dyx::rpp;
int main() {
  snapshot_detail::Sha256 h;
  assert(h.hex()=="e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  h.text("abc");
  assert(h.hex()=="ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  const std::vector<RawPathPoint> path{
    {{1.0,2.0},point_marking,0},
    {{1.05,2.0},point_pass_through,-1},
    {{2.0,2.0},point_marking,1}
  };
  const std::vector<Point2D> targets{{1.0,2.0},{2.0,2.0}};
  const auto s=signed_snapshot(path,targets);
  assert(s.signature=="74700f4d1017768f083a1b3b4e62000e2c4f16e6b6c3c5a99cb1dca3062aa857");
  assert(valid_path_signature(s.signature));
  assert(!valid_path_signature("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"));
  assert(s.raw_navigation().size()==3);
  assert(s.raw_navigation()[1].marking_index==-1);
}
