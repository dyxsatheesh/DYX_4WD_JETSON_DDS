#include "dyx_px4_dds_bridge/bench_transport_interlock.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
int main() {
  using namespace dyx::bridge;
  BenchTransportInterlock gate;
  BenchCommandOutput c{true,false,0.10,0.05,"SIMULATION_CANDIDATE"};
  int checks=0;
  auto check=[&](bool x){assert(x);++checks;};
  auto step=[&](const BenchCommandOutput& input,bool stop=false,bool estop=false,
                bool writer=false,bool hb=true,bool est=true){
    return gate.evaluate(input,stop,estop,writer,hb,est);
  };
  check(gate.latched());
  check(!step(c).candidate_accepted);
  check(!gate.reset_simulation(false,true,true,true,true,true,0,0));
  check(!gate.reset_simulation(true,false,true,true,true,true,0,0));
  check(!gate.reset_simulation(true,true,false,true,true,true,0,0));
  check(!gate.reset_simulation(true,true,true,false,true,true,0,0));
  check(!gate.reset_simulation(true,true,true,true,false,true,0,0));
  check(!gate.reset_simulation(true,true,true,true,true,false,0,0));
  check(!gate.reset_simulation(true,true,true,true,true,true,0.01,0));
  check(!gate.reset_simulation(true,true,true,true,true,true,0,0.01));
  check(!gate.reset_simulation(true,true,true,true,true,true,
                                std::numeric_limits<double>::quiet_NaN(),0));
  check(gate.reset_simulation(true,true,true,true,true,true,0,0));
  auto v=step(c);
  check(v.candidate_accepted && !v.physical_publish_authorized);
  check(v.simulated_forward_mps==0.10 && v.simulated_yaw_rate_radps==0.05);
  check(v.accepted_count==1);
  auto fail=[&](BenchCommandOutput x,bool stop=false,bool estop=false,
              bool writer=false,bool hb=true,bool est=true){
    auto r=step(x,stop,estop,writer,hb,est);
    check(!r.candidate_accepted && r.latched_stop && !r.physical_publish_authorized);
    check(r.simulated_forward_mps==0 && r.simulated_yaw_rate_radps==0);
    check(!step(c).candidate_accepted);
    check(gate.reset_simulation(true,true,true,true,true,true,0,0));
  };
  fail(c,true);
  fail(c,false,true);
  fail(c,false,false,true);
  fail(c,false,false,false,false);
  fail(c,false,false,false,true,false);
  auto q=c;q.candidate_valid=false;fail(q);
  q=c;q.staged_speed_mps=std::numeric_limits<double>::quiet_NaN();fail(q);
  q=c;q.staged_speed_mps=std::numeric_limits<double>::infinity();fail(q);
  q=c;q.staged_speed_mps=-1.001;fail(q);
  q=c;q.staged_yaw_rate_radps=0.181;fail(q);
  q=c;q.staged_yaw_rate_radps=std::numeric_limits<double>::quiet_NaN();fail(q);
  v=step(c);
  check(v.candidate_accepted && !v.physical_publish_authorized);
  check(v.accepted_count==2);
  check(!step(c,true).candidate_accepted);
  check(gate.latched());
  std::cout<<"PASS: "<<checks<<" simulated transport/latch checks; hardware output permanently false\
";
}
