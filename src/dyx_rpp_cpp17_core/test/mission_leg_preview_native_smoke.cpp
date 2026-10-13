#include "dyx_rpp_cpp17_core/mission_leg_preview.hpp"
#include <cstdlib>
#define CHECK(cond) do { if (!(cond)) { std::cerr << "CHECK FAILED: " << #cond << " at " << __FILE__ << ":" << __LINE__ << "\n"; std::exit(1); } } while (false)
#include <cmath>
#include <iostream>
using namespace dyx::rpp;

static MissionLegPreviewInput sample(double t, double x, double y = 0.0) {
  MissionLegPreviewInput a{};
  a.now_sec=t; a.dt_sec=0.05; a.position_sample_time_sec=t;
  a.marking_reference_enu={x,y};
  a.rover_yaw_enu_rad=0.0;
  a.measured_forward_speed_mps=0.10;
  a.position_derived_speed_mps=0.10;
  a.measured_yaw_rate_enu_radps=0.0;
  a.marking_reference_calibrated=true;
  a.telemetry_fresh=true; a.rtk_fixed=true;
  a.mission_enabled=true; a.battery_ok=true;
  a.obstacle_clear=true; a.emergency_stop=false;
  return a;
}
int main() {
  const std::vector<RawPathPoint> points={{{0.,0.},point_pass_through,-1},
                                           {{2.,0.},point_marking,0}};
  MissionLegPreview leg(points,1);
  bool saw_forward=false;
  for(int i=0;i<30;++i) {
    const auto out=leg.step(sample(0.05*i, 0.02*i));
    if(!out.valid) {
      std::cerr<<"failed at "<<i<<" reason="<<out.reason<<"\n";
      return 1;
    }
    CHECK(out.forward_preview_mps>=0.0 && out.forward_preview_mps<=1.0);
    CHECK(out.yaw_rate_enu_preview_radps==0.0);
    CHECK(out.goal_identity=="P0001");
    saw_forward=saw_forward || out.forward_preview_mps>0.0;
    CHECK(!out.terminal_certified);
  }
  CHECK(saw_forward);
  bool certified=false;
  for(int i=30;i<54;++i) {
    auto s=sample(0.05*i,1.997);
    s.measured_forward_speed_mps=0.0;
    s.position_derived_speed_mps=0.0;
    const auto out=leg.step(s);
    if(!out.valid) {
      std::cerr<<"terminal failed at "<<i<<" reason="<<out.reason<<"\n";
      return 2;
    }
    CHECK(out.hold_zero && out.forward_preview_mps==0.0);
    CHECK(out.yaw_rate_enu_preview_radps==0.0);
    CHECK(std::abs(out.radial_error_m-0.003)<1e-8);
    if(out.terminal_certified) certified=true;
  }
  CHECK(certified && !leg.latched());
  std::cout<<"PASS forward-only mission-leg geometry/guidance/speed/20mm stop preview\n";
}
