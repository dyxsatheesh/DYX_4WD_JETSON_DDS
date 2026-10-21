#pragma once
// D4B mission-owner spray validation SHADOW. Diagnostic only, no ROS/PX4 API.
// Independent RAW RTK survey, stopped proof, physical OFF feedback, unique
// ACK correlation and deployment safety are not yet commissioned.
#include "dyx_rpp_cpp17_core/mission_sequence.hpp"
#include <string>

namespace dyx::rpp {
struct SprayIntentShadowInput {
  MissionStage stage{MissionStage::unloaded};
  std::string active_terminal_identity{};
  std::string certified_terminal_identity{};
  bool mission_owner_verified{false};
  bool terminal_estimator_stop_certified{false};
  bool independent_raw_rtk_survey_pass{false};
  bool physical_estop_clear_verified{false};
  bool physical_off_feedback_verified{false};
  bool durable_journal_ready{false};
  bool uniquely_correlated_ack_available{false};
  bool operator_lease_verified{false};
  bool spray_hardware_commissioned{false};
};
struct SprayIntentShadowResult {
  bool eligibility_diagnostic{false};
  bool hold_zero{true};
  bool dispatch_authorized{false};
  bool spray_authorized{false};
  const char* reason{"SPRAY_NOT_REQUESTED"};
};
inline SprayIntentShadowResult evaluate_spray_intent_shadow(
    const SprayIntentShadowInput& e) noexcept {
  SprayIntentShadowResult out{};
  if (e.stage != MissionStage::spray_ack) {
    out.reason="MISSION_NOT_AWAITING_SPRAY_ACK";
    return out;
  }
  if (!e.mission_owner_verified || e.active_terminal_identity.empty() ||
      e.active_terminal_identity != e.certified_terminal_identity) {
    out.reason="MISSION_OR_IDENTITY_UNVERIFIED";
    return out;
  }
  if (!e.terminal_estimator_stop_certified || !e.independent_raw_rtk_survey_pass) {
    out.reason="STOP_OR_RAW_GNSS_SURVEY_UNVERIFIED";
    return out;
  }
  if (!e.physical_estop_clear_verified || !e.physical_off_feedback_verified ||
      !e.operator_lease_verified || !e.spray_hardware_commissioned) {
    out.reason="PHYSICAL_SAFETY_UNCOMMISSIONED";
    return out;
  }
  if (!e.durable_journal_ready || !e.uniquely_correlated_ack_available) {
    out.reason="DDS_TRANSACTION_PROOFS_MISSING";
    return out;
  }
  out.eligibility_diagnostic=true;
  out.reason="ELIGIBLE_DIAGNOSTIC_ONLY";
  // Not a command capability: intentionally constant regardless of inputs.
  return out;
}
} // namespace dyx::rpp
