#pragma once
// Non-actuating mapping of raw GNSS survey truth to MissionSequencer inputs.
// The caller must separately provide a valid terminal certificate, mission
// safety gates and spray acknowledgements. This adapter grants NONE of those.
#include "dyx_rpp_cpp17_core/mission_sequence.hpp"
#include "dyx_rpp_cpp17_core/raw_survey_accuracy.hpp"
#include <cmath>
#include <stdexcept>
#include <string>

namespace dyx::rpp {

inline void apply_raw_survey_evidence(MissionSequenceInput &input,
                                     const RawSurveyDecision &decision,
                                     const std::string &active_terminal_identity) {
  if (active_terminal_identity.empty() || decision.terminal_identity != active_terminal_identity)
    throw std::invalid_argument("cross-goal raw GNSS evidence identity");
  input.survey_identity = active_terminal_identity;
  input.survey = SurveyVerdict::waiting;
  input.survey_validated_raw_fixed = false;
  input.survey_radial_error_m = 0.0;
  input.survey_last_sample_sec = 0.0;
  switch (decision.verdict) {
    case RawSurveyVerdict::waiting:
      return;
    case RawSurveyVerdict::fail:
      input.survey = SurveyVerdict::fail;
      return;
    case RawSurveyVerdict::pass:
      if (!decision.validated_raw_fixed || !decision.radial_error_m ||
          !decision.last_sample_monotonic_sec ||
          !std::isfinite(*decision.radial_error_m) ||
          *decision.radial_error_m < 0.0 ||
          !std::isfinite(*decision.last_sample_monotonic_sec) ||
          *decision.last_sample_monotonic_sec < 0.0 ||
          *decision.radial_error_m > decision.tolerance_m)
        throw std::invalid_argument("invalid PASS raw GNSS survey evidence");
      input.survey = SurveyVerdict::pass;
      input.survey_validated_raw_fixed = true;
      input.survey_radial_error_m = *decision.radial_error_m;
      input.survey_last_sample_sec = *decision.last_sample_monotonic_sec;
      return;
  }
  throw std::logic_error("unknown raw survey verdict");
}
} // namespace dyx::rpp
