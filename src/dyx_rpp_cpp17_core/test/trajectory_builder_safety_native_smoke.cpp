#include "dyx_rpp_cpp17_core/trajectory_builder.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace dyx::rpp;
template <class F> void must_reject(F f) {
  bool rejected=false;
  try { f(); } catch(const std::invalid_argument &) { rejected=true; }
  catch(const std::length_error &) { rejected=true; }
  assert(rejected);
}
int main() {
  NativeTrajectoryBuilder gen;
  must_reject([&]{gen.build({{0,0}});});
  must_reject([&]{gen.build({{0,0},{0,0}});});
  must_reject([&]{gen.build({{0,0},{0.00001,0}});});
  must_reject([&]{gen.build({{0,0},{std::numeric_limits<double>::quiet_NaN(),1}});});
  must_reject([&]{gen.build({{0,0},{10001,1}});});
  must_reject([&]{gen.build({{0,0},{std::numeric_limits<double>::infinity(),1}});});
  TrajectoryBuildConfig invalid;
  invalid.interpolation_spacing_m=0;
  must_reject([&]{NativeTrajectoryBuilder{invalid};});
  invalid=TrajectoryBuildConfig{};
  invalid.extension_enabled=true;
  invalid.row_transfer_min_angle_deg=145;
  invalid.row_transfer_max_angle_deg=135;
  must_reject([&]{NativeTrajectoryBuilder{invalid};});
  invalid=TrajectoryBuildConfig{};
  invalid.maximum_marking_points=2;
  invalid.maximum_navigation_points=5;
  must_reject([&]{NativeTrajectoryBuilder(invalid).build({{0,0},{1,0}});});
  invalid=TrajectoryBuildConfig{};
  invalid.maximum_marking_points=2;
  must_reject([&]{NativeTrajectoryBuilder(invalid).build({{0,0},{1,0},{2,0}});});
  // Extension beyond bounded metric frame is a hard rejection.
  invalid=TrajectoryBuildConfig{};
  invalid.extension_enabled=true;
  invalid.max_abs_coordinate_m=12.0;
  must_reject([&]{NativeTrajectoryBuilder(invalid).build({{0,0},{0,10},{1,10},{1,0}});});
  std::cout << "trajectory_builder_safety_native_smoke passed\n";
}
