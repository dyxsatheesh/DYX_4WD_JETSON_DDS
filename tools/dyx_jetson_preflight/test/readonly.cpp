#include "dyx_jetson_readonly_dds/status_guard.hpp"
#include <cassert>
#include <limits>
int main(){
  dyx::readonly_dds::State s;
  assert(!s.output_allowed()&&!s.observation_fresh(1));
  s.observe_status(true,true,1.0);s.observe_position(true,1.0);
  assert(s.observation_fresh(1.2));
  assert(!s.output_allowed());
  assert(!s.observation_fresh(1.6));
  s.observe_status(false,false,2.0);s.observe_position(false,2.0);
  assert(!s.observation_fresh(2.0));
  s.observe_position(true,std::numeric_limits<double>::quiet_NaN());
  assert(!s.observation_fresh(3.0));
}
