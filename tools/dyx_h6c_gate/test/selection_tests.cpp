#include "dyx_mission_cpp17_preview/selection.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
using dyx::mission_preview::validate_selection;
template<class F> static bool rejects(F&& f){try{f();return false;}catch(const std::exception&){return true;}}
int main(){
  const auto plan=validate_selection({0,0,1,0,1,2},2);
  assert(plan.markings.size()==3 && plan.selected_index==2);
  assert(plan.chosen.north_m==1 && plan.chosen.east_m==2);
  assert(!plan.physical_command_permitted);
  assert(rejects([]{validate_selection({},0);}));
  assert(rejects([]{validate_selection({0,0},0);}));
  assert(rejects([]{validate_selection({0,0,1},0);}));
  assert(rejects([]{validate_selection({0,0,1,1},-1);}));
  assert(rejects([]{validate_selection({0,0,1,1},2);}));
  assert(rejects([]{validate_selection({0,0,0.0001,0},0);}));
  assert(rejects([]{validate_selection({0,0,std::numeric_limits<double>::quiet_NaN(),1},0);}));
  assert(rejects([]{validate_selection({0,0,10000.1,0},0);}));
  assert(rejects([]{validate_selection({0,0,std::numeric_limits<double>::infinity(),0},0);}));
  return 0;
}
