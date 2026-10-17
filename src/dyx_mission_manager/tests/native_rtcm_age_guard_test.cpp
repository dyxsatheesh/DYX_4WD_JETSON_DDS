#include "dyx_mission_manager/native_rtcm_age_guard.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using dyx::http_backend::NativeRtcmAgeGuard;
int main() {
  NativeRtcmAgeGuard missing;
  assert(!missing.receiver_corrections_fresh(2.0));
  assert(!missing.receiver_report_connected());
  assert(std::isnan(missing.effective_age_sec(2.0)));
  NativeRtcmAgeGuard good;
  good.observe_receiver_report(10.,100,19.5,true);
  assert(good.receiver_corrections_fresh(10.2));
  assert(!good.receiver_corrections_fresh(10.6));
  assert(!good.receiver_corrections_fresh(11.01));
  good.observe_receiver_report(11.1,101,0.1,true);
  assert(good.receiver_corrections_fresh(11.1));
  good.observe_receiver_report(11.2,101,0.1,true);
  assert(good.source_fault() && !good.receiver_corrections_fresh(11.2));
  NativeRtcmAgeGuard fake;
  fake.observe_receiver_report(1,1,0.1,false);
  assert(fake.source_fault() && !fake.receiver_corrections_fresh(1));
  NativeRtcmAgeGuard invalid;
  invalid.observe_receiver_report(1,1,-1,true);
  assert(!invalid.receiver_corrections_fresh(1));
  invalid.observe_receiver_report(1.1,2,std::numeric_limits<double>::quiet_NaN(),true);
  assert(!invalid.receiver_corrections_fresh(1.1));
  invalid.observe_receiver_report(1.2,3,20.01,true);
  assert(!invalid.receiver_corrections_fresh(1.2));
  std::cout << "PASS: receiver age 20s boundary, monotonic aging, stale report, invalid age, unverified provenance and replay fail closed\n";
}
