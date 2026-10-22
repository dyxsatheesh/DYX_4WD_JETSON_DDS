#pragma once
// B118: authoritative *diagnostic* bridge between the mission stage checker
// and the session/evidence authority model. No control grants or actuation.
#include "dyx_mission_manager/native_motion_authority.hpp"
#include "dyx_mission_manager/native_mission_transition_gate.hpp"
#include <cmath>

namespace dyx::http_backend {
struct NativeAuthorityDispatchDecision {
  MissionTransitionDecision transition{};
  AuthorityDecision authority{};
};
inline NativeAuthorityDispatchDecision assess_authority_dispatch(
    const NativeMotionAuthority& authority,
    const dyx::rpp::BackendMissionApiRequest& request,
    dyx::rpp::MissionStage stage, bool mailbox_stop_latched,
    double now_monotonic_sec) {
  NativeAuthorityDispatchDecision out{};
  out.transition = assess_mission_transition(
      request, stage, mailbox_stop_latched);
  // A latched STOP on the mailbox takes precedence over every lease/evidence.
  if (mailbox_stop_latched) {
    out.authority.reason = "MAILBOX_STOP_LATCHED";
    return out;
  }
  out.authority = authority.assess(request, stage, now_monotonic_sec);
  // Software-ready is a reporting observation, never dispatch authorization.
  out.transition.dispatch_authorized = false;
  out.authority.motion_authorized = false;
  return out;
}
} // namespace dyx::http_backend
