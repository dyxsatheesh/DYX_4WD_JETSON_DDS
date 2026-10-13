#include "dyx_rpp_cpp17_core/path_index.hpp"
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using namespace dyx::rpp;
namespace {
struct Parser {
  int argc;
  char **argv;
  int i{2};
  double dbl() { if(i>=argc) throw std::invalid_argument("missing argument"); return std::stod(argv[i++]); }
  int integer() {return static_cast<int>(dbl());}
  std::optional<int> opt_integer(){const int v=integer();return v<0?std::nullopt:std::optional<int>(v);}
  std::optional<double> opt_double(){const double v=dbl();return v==-999.0?std::nullopt:std::optional<double>(v);}
};
std::vector<double> flatten(const PathProjection &p) {
  return {static_cast<double>(p.segment_index.value_or(-1)),p.projected_s,p.progress_s,
    p.point.x,p.point.y,p.signed_cross_track_m,p.distance_m,p.t,
    static_cast<double>(p.used_full_reacquire),p.remaining_to_active_stop_m,
    p.remaining_path_m,p.next_corner_distance_m.value_or(-999),
    p.next_corner_angle_rad.value_or(-999),p.next_leg_heading_rad.value_or(-999),
    static_cast<double>(p.monotonic_clamped)};
}
} // namespace
int main(int argc,char **argv) {
 try {
  if (argc<3) throw std::invalid_argument("required: mode, N, raw points ...");
  const std::string mode(argv[1]);
  Parser a{argc,argv};
  const int n=a.integer();
  if(n<=0 || n>20000) throw std::invalid_argument("raw point count out of range");
  std::vector<RawPathPoint> raw;
  for(int i=0;i<n;++i){
    const double x=a.dbl(),y=a.dbl();
    const int type=a.integer(),mark=a.integer();
    raw.push_back({{x,y},type,mark});
  }
  const int start=a.integer(),stop=a.integer();
  const auto geo=PathGeometryIndex::build(raw);
  const auto span=geo.active_span(stop,start<0?std::nullopt:std::optional<int>(start));
  ProjectOptions opt;
  if(mode=="index"){
    const Point2D query{a.dbl(),a.dbl()};
    opt.hint_segment_index=a.opt_integer();
    opt.reference_s=a.opt_double();
    opt.back_window_segments=a.integer();
    opt.forward_window_segments=a.integer();
    opt.max_backward_jump_m=a.dbl();
    opt.max_forward_jump_m=a.opt_double();
    opt.full_reacquire_distance_m=a.opt_double();
    const double lookahead=a.dbl();
    const auto p=geo.project(query,span,opt);
    const auto t=geo.lookahead_target(p.progress_s,lookahead,span);
    std::vector<double> res={geo.total_length,static_cast<double>(geo.segments.size()),
      static_cast<double>(geo.corners.size()),static_cast<double>(geo.semantic_anchors.size()),
      span.start_s,span.stop_s,static_cast<double>(span.first_segment_index.value_or(-1)),
      static_cast<double>(span.last_segment_index.value_or(-1))};
    for(double v:flatten(p))res.push_back(v);
    res.insert(res.end(),{static_cast<double>(t.segment_index.value_or(-1)),t.t,t.point.x,
      t.point.y,t.s,t.heading_rad.value_or(-999)});
    for(const auto &c:geo.corners){res.push_back(static_cast<double>(c.raw_index));res.push_back(c.s);res.push_back(c.turn_angle_rad);}
    std::cout<<std::setprecision(17);
    for(size_t i=0;i<res.size();++i)std::cout<<(i?" ":"")<<res[i];
    std::cout<<'\n';
  }else if(mode=="track"){
    const Point2D q1{a.dbl(),a.dbl()},q2{a.dbl(),a.dbl()};
    opt.back_window_segments=a.integer();
    opt.forward_window_segments=a.integer();
    opt.max_backward_jump_m=a.dbl();
    opt.max_forward_jump_m=a.opt_double();
    opt.full_reacquire_distance_m=a.opt_double();
    GeometryProgressTracker tracker(geo);
    const auto p1=tracker.update(q1,span,opt);
    const auto p2=tracker.update(q2,span,opt);
    const auto one=flatten(p1),two=flatten(p2);
    std::cout<<std::setprecision(17);
    bool first=true;
    for(const auto &v: {one,two})for(double x:v){std::cout<<(first?"":" ")<<x;first=false;}
    std::cout<<'\n';
  }else throw std::invalid_argument("unknown mode");
  if(a.i!=argc)throw std::invalid_argument("unexpected trailing args");
  return 0;
 } catch(const std::exception &e) {std::cerr<<"geometry index probe: "<<e.what()<<'\n';return 2;}
}
