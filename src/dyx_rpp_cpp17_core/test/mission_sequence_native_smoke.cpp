#include "dyx_rpp_cpp17_core/mission_sequence.hpp"
#include <cassert>
#include <string>
#include <vector>
using namespace dyx::rpp;

static MissionSequenceInput frame(double time, std::string identity) {
  MissionSequenceInput s{};
  s.monotonic_time_sec = time;
  s.motion_health_authorized = true;
  s.emergency_stop = false;
  s.terminal_present = true;
  s.terminal_identity = std::move(identity);
  s.terminal_certificate_valid = true;
  s.terminal_zero_latched = true;
  s.terminal_stationary = true;
  s.terminal_radial_error_m = 0.011;
  s.measured_forward_speed_mps = 0.0;
  return s;
}

int main() {
  const std::string a = "RUN:42|PATH:ABC|GOAL:P0001|INSTANCE:A";
  const std::string b = "RUN:42|PATH:ABC|GOAL:P0002|INSTANCE:B";
  const std::string c = "RUN:42|PATH:ABC|GOAL:P0003|INSTANCE:C";
  MissionSequencer mission;
  mission.load({{1, 0, "P0001", a}, {2, 1, "P0002", b},
                {3, 2, "P0003", c}}, MissionMode::automatic);
  assert(mission.status().hold_zero);
  mission.start(true);
  auto s = frame(1.0, a);
  auto out = mission.step(s);
  assert(out.event == MissionEvent::stop_verified);
  assert(out.hold_zero && out.stage == MissionStage::verify_hold);
  s = frame(1.5, a);
  s.survey = SurveyVerdict::pass;
  s.survey_identity = a;
  s.survey_validated_raw_fixed = true;
  s.survey_last_sample_sec = 1.4;
  s.survey_radial_error_m = 0.021;
  out = mission.step(s);
  assert(out.stage == MissionStage::verify_hold && !out.request_spray);
  s = frame(4.0, a);
  out = mission.step(s);
  assert(out.event == MissionEvent::request_spray && out.request_spray);
  assert(out.hold_zero && out.completed == 0);
  s = frame(4.1, a);
  out = mission.step(s);
  assert(out.stage == MissionStage::spray_ack && !out.request_spray);
  s = frame(4.2, a);
  s.spray_result = SprayVerdict::success;
  s.spray_identity = a;
  s.spray_received_sec = 4.2;
  out = mission.step(s);
  assert(out.event == MissionEvent::point_resolved);
  assert(out.completed == 1 && out.stage == MissionStage::resolved);
  out = mission.step(frame(4.3, a));
  assert(out.event == MissionEvent::advanced && out.current_marking == 1);
  s = frame(5.0, b);
  s.terminal_certificate_valid = false;
  s.rpp_reported_missed = true;
  out = mission.step(s);
  assert(out.failed == 1 && out.completed == 1 && out.stage == MissionStage::resolved);
  out = mission.step(frame(5.1, b));
  assert(out.current_marking == 2 && out.stage == MissionStage::approaching);
  mission.skip();
  out = mission.status();
  assert(out.skipped == 1 && out.stage == MissionStage::resolved);
  out = mission.step(frame(5.2, c));
  assert(out.event == MissionEvent::mission_finished);
  assert(out.stage == MissionStage::finished && out.total == 3);
  assert(out.completed + out.failed + out.skipped == out.total);
  assert(out.hold_zero && !out.allow_next_leg_preview);
  assert(mission.records()[0].result == MarkingResult::completed);
  assert(mission.records()[1].result == MarkingResult::failed);
  assert(mission.records()[2].result == MarkingResult::skipped);
  return 0;
}
