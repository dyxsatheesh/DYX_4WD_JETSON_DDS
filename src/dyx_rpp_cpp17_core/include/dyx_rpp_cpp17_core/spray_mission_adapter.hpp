#pragma once
// Non-actuating binding from one completed spray RELEASE acknowledgement to
// the exact mission semantic identity. NEVER fabricate spray success from a
// PRESS acknowledgement, timeout, elapsed duration or GPIO/PWM intent alone.
#include "dyx_rpp_cpp17_core/mission_sequence.hpp"
#include "dyx_rpp_cpp17_core/spray_transaction.hpp"
namespace dyx::rpp {
inline bool bind_spray_release_to_mission(const SprayOutput &spray,
                                        MissionSequenceInput &mission) {
  if (!spray.release_acknowledged || spray.completion==SprayCompletion::none ||
      spray.completed_identity.empty() || mission.terminal_identity!=spray.completed_identity ||
      !mission.terminal_present || !std::isfinite(spray.completion_time_sec) ||
      spray.completion_time_sec<0 ||
      spray.completion_time_sec>mission.monotonic_time_sec) return false;
  mission.spray_identity=spray.completed_identity;
  mission.spray_received_sec=spray.completion_time_sec;
  mission.spray_result=spray.completion==SprayCompletion::completed ?
      SprayVerdict::success:SprayVerdict::failed;
  return true;
}
} // namespace dyx::rpp
