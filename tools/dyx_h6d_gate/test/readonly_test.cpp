#include "dyx_safety_cpp17_observer/read_only_contract.hpp"
#include <cassert>
#include <initializer_list>
#include <string>
int main() {
  for (bool fault : {false,true}) {
    for (bool battery : {false,true}) {
      for (bool flt : {false,true}) {
        const auto a=dyx::h6d::observe_only(fault,battery,flt,true,"READY");
        assert(!a.physical_command_permitted);
        assert(!a.mission_transition_permitted);
        assert(!a.spray_permitted);
        assert(!a.correction_source_verified);
        assert(!a.obstacle_source_verified);
        assert(a.battery_critical_latched==battery);
        assert(a.rtk_float_confirming==flt);
        assert(!a.reason.empty());
      }
    }
  }
  assert(dyx::h6d::observe_only(false,false,false,false,nullptr).reason==
         "SAFETY_MODEL_REASON_MISSING");
  return 0;
}
