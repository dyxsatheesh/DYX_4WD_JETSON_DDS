#include "dyx_mission_manager/native_rtk_float_guard.hpp"
#include <cassert>
#include <iostream>
using dyx::http_backend::NativeRtkFloatGuard;
int main() {
 NativeRtkFloatGuard g;
 assert(g.pause_requested(0) && !g.corrections_fresh_verified());
 g.observe(1,100,6,false,false);
 assert(g.fixed(1) && !g.pause_requested(1));
 g.observe(2,101,5,false,false);
 assert(!g.float_over_five_seconds(6.9));
 // At >1s without a new sample, GNSS must be treated as stale, not held FIX.
 assert(g.pause_requested(6.9));
 for(int i=1;i<=50;i++) g.observe(2+0.1*i,101+i,5,false,false);
 assert(!g.float_over_five_seconds(7.0));
 g.observe(7.1,152,5,false,false);
 assert(g.float_over_five_seconds(7.1) && g.pause_requested(7.1));
 g.observe(7.2,153,6,false,false);
 assert(g.fixed(7.2) && !g.pause_requested(7.2));
 g.observe(7.3,153,6,false,false);
 assert(g.fault() && g.pause_requested(7.3));
 NativeRtkFloatGuard gap;
 gap.observe(1,1,5,false,false);
 gap.observe(9,2,5,false,false);
 assert(!gap.float_over_five_seconds(9));
 gap.observe(9.2,3,6,true,false);
 assert(!gap.fixed(9.2) && gap.pause_requested(9.2) == false);
 assert(!gap.corrections_fresh_verified());
 std::cout << "PASS: FIX/FLOAT >5s, immediate FIX reset, stale GNSS hold, reordered sample fault, no RTCM spoofing\n";
}
