// Native parity probe over the REAL C++ path index, no ROS connection.
#include "dyx_rpp_cpp17_core/path_index.hpp"
#include "dyx_rpp_cpp17_core/precision_guidance.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace dyx::rpp;
int main(int argc, char **argv) {
  try {
    int arg=1;
    auto num=[&]()->double {
      if (arg>=argc) throw std::invalid_argument("missing argument");
      const auto value=std::stod(argv[arg++]);
      require_finite(value,"probe input");
      return value;
    };
    auto integer=[&]()->int {return static_cast<int>(num());};
    const Point2D rover{num(),num()};
    const double yaw=num(), speed=num();
    PrecisionGuidanceConfig cfg;
    cfg.lookahead={num(),num(),num(),num()};
    cfg.moving_bearing_cone_rad=num();
    const int start=integer(),stop=integer(),n=integer();
    if(n<=0 || n>5000) throw std::invalid_argument("invalid path size");
    std::vector<RawPathPoint> points;
    points.reserve(static_cast<size_t>(n));
    for(int k=0;k<n;++k) {
      const Point2D p{num(),num()};
      const int kind=integer(),mark=integer();
      points.push_back({p,kind,mark});
    }
    if(arg!=argc) throw std::invalid_argument("unexpected extra probe args");
    const auto geometry=PathGeometryIndex::build(points);
    const auto span=geometry.active_span(stop,start);
    const auto projection=geometry.project(rover,span);
    const auto g=compute_precision_guidance(cfg,geometry,projection,span,rover,yaw,speed);
    std::cout<<std::setprecision(17);
    std::cout<<g.lookahead_distance_m<<' '<<g.lookahead_target_s<<' '
             <<g.lookahead_segment_index.value_or(-1)<<' '
             <<g.lookahead_point.x<<' '<<g.lookahead_point.y<<' '
             <<g.steering_target_point.x<<' '<<g.steering_target_point.y<<' '
             <<g.actual_steering_target_distance_m<<' '
             <<int(g.endpoint_extension_used)<<' '<<g.endpoint_extension_distance_m<<' '
             <<int(g.target_behind_rover)<<' '<<g.lookahead_bearing_rad<<' '
             <<g.local_path_heading_rad<<' '<<g.path_heading_error_rad<<' '
             <<g.heading_error_rad<<' '<<g.signed_cross_track_m<<' '
             <<g.desired_movement_bearing_rad<<' '<<g.limited_command_bearing_rad<<' '
             <<g.final_command_correction_rad<<' '
             <<int(g.bearing_clamp_fired)<<' '<<int(g.zero_vector_fallback_used)<<'\n';
    return 0;
  } catch(const std::exception &e) {
    std::cerr<<"guidance probe error: "<<e.what()<<'\n';
    return 2;
  }
}
