#include "dyx_rpp_cpp17_core/mission_leg_preview.hpp"
#include <cstdlib>
#define CHECK(cond) do { if (!(cond)) { std::cerr << "CHECK FAILED: " << #cond << " at " << __FILE__ << ":" << __LINE__ << "\n"; std::exit(1); } } while (false)
#include <cmath>
#include <iostream>
#include <limits>
using namespace dyx::rpp;

static std::vector<RawPathPoint> path() {
  return {{{0.0,0.0},point_pass_through,-1},{{1.0,0.0},point_marking,0}};
}
static MissionLegPreviewInput ready() {
  MissionLegPreviewInput a{};
  a.now_sec=1.0; a.dt_sec=0.05; a.position_sample_time_sec=1.0;
  a.marking_reference_enu={0,0}; a.rover_yaw_enu_rad=0;
  a.position_derived_speed_mps=0.1; a.measured_forward_speed_mps=0.1;
  a.marking_reference_calibrated=true;
  a.telemetry_fresh=true; a.rtk_fixed=true;
  a.mission_enabled=true; a.battery_ok=true; a.obstacle_clear=true;
  a.emergency_stop=false;
  return a;
}
int main() {
  {
    MissionLegPreview rover(path(),1);
    auto s=ready();s.marking_reference_calibrated=false;
    CHECK(!rover.step(s).valid && rover.latched());
    // Reinstating calibration does not silently clear a mission lockout.
    CHECK(!rover.step(ready()).valid);
  }
  {
    MissionLegPreview rover(path(),1);
    auto s=ready();s.emergency_stop=true;
    const auto r=rover.step(s);
    CHECK(!r.valid && r.forward_preview_mps==0 && r.yaw_rate_enu_preview_radps==0);
  }
  {
    MissionLegPreview rover(path(),1);
    auto s=ready();s.rtk_fixed=false;
    CHECK(!rover.step(s).valid && rover.latched());
  }
  {
    MissionLegPreview rover(path(),1);
    auto s=ready();s.measured_forward_speed_mps=-0.1;
    // Forward-only preview rejects measured reverse travel, not only reverse commands.
    auto r=rover.step(s);
    CHECK(!r.valid && rover.latched() && r.forward_preview_mps==0);
  }
  {
    MissionLegPreview rover(path(),1);
    auto s=ready();s.position_sample_time_sec=0.3;
    CHECK(!rover.step(s).valid && rover.latched());
  }
  {
    MissionLegPreview rover(path(),1);
    auto s=ready();s.rover_yaw_enu_rad=std::numeric_limits<double>::quiet_NaN();
    CHECK(!rover.step(s).valid && rover.latched());
  }
  {
    MissionLegPreview rover(path(),1);
    auto s=ready();s.rover_yaw_enu_rad=0.6;
    // No permission for a 34 degree turn: speed and yaw must both be zero.
    auto r=rover.step(s);
    CHECK(r.valid && r.hold_zero==false); // hold_zero is reserved for terminal stop
    CHECK(r.forward_preview_mps==0.0 && r.yaw_rate_enu_preview_radps==0.0);
  }
  {
    bool threw=false;
    try { MissionLegPreview invalid({{{0.,0.},point_pass_through,-1}},0); }
    catch (const std::invalid_argument &) { threw=true; }
    CHECK(threw);
  }
  std::cout<<"PASS mission-leg fail-closed preview with calibrated marking reference\n";
}
