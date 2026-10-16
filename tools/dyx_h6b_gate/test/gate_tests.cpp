#include "dyx_rpp_cpp17_native_preview/admission.hpp"
#include <cassert>
#include <limits>
int main() {
  using dyx::preview::Admission;
  using dyx::preview::Evidence;
  Evidence e;
  e.now_s = 10.0;
  e.preview_enabled = false;
  Admission disabled;
  assert(!disabled.inspect(e).preview_permitted);
  e.preview_enabled = true;
  Admission gate;
  assert(!gate.inspect(e).preview_permitted); // missing inputs must wait
  e.position_seen = e.rate_seen = e.goal_seen = true;
  e.estimator_valid = true;
  e.telemetry_finite = true;
  e.position_sample_us = 1000000;
  e.rate_sample_us = 1000020;
  e.goal_north_m = 4.0;
  e.goal_east_m = 3.0;
  const auto good = gate.inspect(e);
  assert(good.preview_permitted && !good.physical_command_permitted);
  e.now_s += 0.02;
  e.rate_age_s = 0.30;
  const auto stale = gate.inspect(e);
  assert(!stale.preview_permitted && stale.fault_latched);
  e.rate_age_s = 0.0;
  e.now_s += 0.02;
  assert(!gate.inspect(e).preview_permitted); // cannot self-reenable
  assert(!gate.inspect(e).physical_command_permitted);
  Admission reset;
  e.now_s = 10.0;
  e.reset_detected = true;
  assert(reset.inspect(e).fault_latched);
  Admission wrong_frame;
  e.reset_detected = false;
  e.invalid_goal = true;
  assert(wrong_frame.inspect(e).fault_latched);
  Admission nonfinite;
  e.invalid_goal = false;
  e.goal_north_m = std::numeric_limits<double>::quiet_NaN();
  assert(nonfinite.inspect(e).fault_latched);
  Admission no_sample;
  e.goal_north_m = 1.0;
  e.position_sample_us = 0;
  assert(no_sample.inspect(e).fault_latched);
  Admission skew;
  e.position_sample_us = 1000000;
  e.rate_sample_us = 2001000;
  assert(skew.inspect(e).fault_latched);
  Admission time_order;
  e.rate_sample_us = 1000000;
  assert(time_order.inspect(e).preview_permitted);
  e.now_s = 9.0;
  assert(time_order.inspect(e).fault_latched);
  return 0;
}
