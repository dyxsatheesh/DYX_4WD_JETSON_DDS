#include "dyx_rpp_cpp17_core/path_index.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace dyx::rpp;
namespace {
void expect(bool yes,const char *msg){if(!yes)throw std::runtime_error(msg);}
void near(double x,double y,const char *msg){expect(std::abs(x-y)<1e-12,msg);}
}
int main(){
 try {
  const std::vector<RawPathPoint> raw={
    {{0,0},0,-1},{{2,0},1,-1},{{2,0},0,-1},
    {{2,2},2,0},{{4,2},2,1}};
  const auto g=PathGeometryIndex::build(raw);
  expect(g.raw_points.size()==5,"authoritative raw point count changed");
  expect(g.segments.size()==3,"duplicate raw point became segment");
  expect(g.corners.size()==2,"90 degree corners not preserved");
  expect(g.semantic_anchors.size()==3,"semantic waypoint lost");
  expect(g.semantic_anchors[1].identity=="P0001","marking identity incorrect");
  expect(g.corners[0].raw_index==1,"corner did not bind to dummy anchor");
  const auto span=g.active_span(3,0);
  near(span.stop_s,4.0,"span stop incorrect");
  const auto q=g.project({2.02,1.0},span);
  expect(q.segment_index.value()==1,"projected onto wrong segment");
  expect(q.signed_cross_track_m<0.0,"cross-track sign inverted");
  near(q.projected_s,3.0,"projection progress incorrect");
  const auto target=g.lookahead_target(q.projected_s,2.0,span);
  near(target.point.y,2.0,"lookahead did not respect active stop");
  expect(g.next_corner(2.0)==&g.corners[1],"strictly future corner selection incorrect");
  GeometryProgressTracker tracker(g);
  const auto forward=tracker.update({2.0,1.0},span);
  const auto backward=tracker.update({2.0,0.98},span);
  expect(backward.monotonic_clamped,"backtracking did not clamp progress");
  near(backward.progress_s,forward.progress_s,"monotonic progress moved backwards");
  bool invalid=false;
  try {PathGeometryIndex::build({{{0,0},2,1}});}catch(const std::invalid_argument&){invalid=true;}
  expect(invalid,"non-sequential marking indices were accepted");
  invalid=false;
  try {g.active_span(4,3);}catch(const std::invalid_argument&){invalid=true;}
  expect(!invalid,"valid bounded span rejected");
  invalid=false;
  try {g.active_span(1,3);}catch(const std::invalid_argument&){invalid=true;}
  expect(invalid,"reverse active span accepted");
  std::cout<<"PASS: native C++17 path index smoke and invariants\n";
  return 0;
 }catch(const std::exception&e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
