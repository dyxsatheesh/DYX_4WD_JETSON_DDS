#pragma once
// D1D: deterministic, non-actuating mission command transition validator.
// Positive actions cannot cross the commissioning boundary in this build.
#include "dyx_rpp_cpp17_core/backend_mission_api.hpp"
#include <cstdint>

namespace dyx::http_backend {
enum class MissionTransitionBlock : std::uint8_t {
  none, untrusted_principal, stop_latched, wrong_stage,
  missing_safety_authority, unknown_action
};
inline const char* transition_block_name(MissionTransitionBlock block) noexcept {
  switch (block) {
    case MissionTransitionBlock::none: return "NONE";
    case MissionTransitionBlock::untrusted_principal: return "UNTRUSTED_PRINCIPAL";
    case MissionTransitionBlock::stop_latched: return "STOP_LATCHED";
    case MissionTransitionBlock::wrong_stage: return "INVALID_STAGE";
    case MissionTransitionBlock::missing_safety_authority: return "SAFETY_AUTHORITY_UNAVAILABLE";
    case MissionTransitionBlock::unknown_action: return "UNKNOWN_COMMAND";
  }
  return "UNKNOWN_COMMAND";
}
struct MissionTransitionDecision {
  MissionTransitionBlock block{MissionTransitionBlock::unknown_action};
  bool eligible_stage{false};
  bool dispatch_authorized{false}; // Must remain false before safety commissioning.
};
inline MissionTransitionDecision assess_mission_transition(
    const dyx::rpp::BackendMissionApiRequest& request,
    dyx::rpp::MissionStage stage,
    bool stop_latched) noexcept {
  using namespace dyx::rpp;
  MissionTransitionDecision result{};
  if (!request.principal.authenticated || !request.principal.owns_control_lease) {
    result.block = MissionTransitionBlock::untrusted_principal;
    return result;
  }
  if (stop_latched || stage == MissionStage::stopped || stage == MissionStage::fault) {
    result.block = MissionTransitionBlock::stop_latched;
    return result;
  }
  if (request.method != "POST") {
    result.block = MissionTransitionBlock::unknown_action;
    return result;
  }
  result.eligible_stage =
      (request.path == kBackendMissionStartPath && stage == MissionStage::ready) ||
      (request.path == kBackendMissionResumePath && stage == MissionStage::paused) ||
      (request.path == kBackendMissionNextPath && stage == MissionStage::await_next) ||
      (request.path == kBackendMissionSkipPath && stage == MissionStage::approaching);
  const bool known = request.path == kBackendMissionStartPath ||
      request.path == kBackendMissionResumePath ||
      request.path == kBackendMissionNextPath ||
      request.path == kBackendMissionSkipPath;
  if (!known) {
    result.block = MissionTransitionBlock::unknown_action;
  } else if (!result.eligible_stage) {
    result.block = MissionTransitionBlock::wrong_stage;
  } else {
    result.block = MissionTransitionBlock::missing_safety_authority;
  }
  return result;
}
} // namespace dyx::http_backend
