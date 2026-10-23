#include "dyx_mission_manager/marking_arrival_policy.hpp"
#include <cassert>
#include <limits>
using namespace dyx::mission::arrival;
int main() {
  assert(phase_a_decision(true, true) == PhaseA::START_VERIFICATION);
  assert(phase_a_decision(true, false) == PhaseA::KEEP_APPROACHING);
  assert(phase_a_decision(false, true, "MISSED") == PhaseA::FAIL_NOW);
  assert(phase_a_decision(false, true, " captured ") == PhaseA::FAIL_NOW);
  assert(phase_a_decision(false, true, "", true) == PhaseA::FAIL_NOW);
  assert(phase_a_decision(false, false, "MISSED") == PhaseA::KEEP_APPROACHING);
  assert(after_fail_mode_action("manual") == FailureAction::WAITING_FOR_NEXT);
  assert(after_fail_mode_action("auto") == FailureAction::AUTO_CONTINUE);
  assert(should_publish_rpp_captured(0.03, 0.05, 0.03, 0.05));
  assert(!should_publish_rpp_captured(0.031, 0.0, 0.03, 0.05));
  assert(!should_publish_rpp_captured(0.01, 0.051, 0.03, 0.05));
  assert(!should_publish_rpp_captured(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.03, 0.05));
}
