// Compile-and-run test for the ROS-free templated C++ precision guidance.
#include "dyx_rpp_cpp17_core/precision_guidance.hpp"
#include <cmath>
#include <optional>
#include <vector>
using namespace dyx::rpp;
struct MockSpan {double stop_s{4.0};std::optional<int> last_segment_index{0};};
struct MockProjection {std::optional<int> segment_index{0};double progress_s{3.95};double signed_cross_track_m{0.02};};
struct MockTarget {std::optional<int> segment_index{0};double t{};Point2D point{4.0,0.0};double s{4.0};std::optional<double> heading_rad{0.0};};
struct MockSegment {double heading_rad{0.0};};
struct MockGeometry {
  std::vector<MockSegment> segments{{0.0}};
  MockTarget lookahead_target(double, double, const MockSpan&) const {return {};}
};
int main() {
  const MockSpan s;const MockGeometry g;const MockProjection p;
  const auto r=compute_precision_guidance(PrecisionGuidanceConfig{},g,p,s,{3.95,0.02},0.0,0.6);
  if (std::abs(r.lookahead_distance_m-0.33)>1e-10) return 1;
  if (!r.endpoint_extension_used) return 2;
  if (std::abs(r.endpoint_extension_distance_m-0.28)>1e-10) return 3;
  if (std::abs(r.steering_target_point.x-4.28)>1e-10) return 4;
  if (r.lookahead_bearing_rad >= 0.0) return 5;
}
