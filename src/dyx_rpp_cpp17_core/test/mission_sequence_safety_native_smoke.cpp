#include "dyx_rpp_cpp17_core/mission_sequence.hpp"
#include <cassert>
#include <limits>
#include <stdexcept>
#include <string>
using namespace dyx::rpp;

static const std::string a = "RUN:A|PATH:P|GOAL:P0001|INSTANCE:X";
static const std::string b = "RUN:A|PATH:P|GOAL:P0002|INSTANCE:Y";
static MissionSequenceInput frame(double t, const std::string &id = a) {
  MissionSequenceInput s{};
  s.monotonic_time_sec = t;
  s.emergency_stop = false;
  s.motion_health_authorized = true;
  s.terminal_present = true;
  s.terminal_identity = id;
  s.terminal_certificate_valid = true;
  s.terminal_zero_latched = true;
  s.terminal_stationary = true;
  s.terminal_radial_error_m = .015;
  return s;
}
static MissionSequencer make_mission(MissionMode mode = MissionMode::automatic) {
  MissionSequencer m;
  m.load({{1, 0, "P0001", a}, {2, 1, "P0002", b}}, mode);
  m.start(true);
  return m;
}
int main() {
  {
    auto m = make_mission();
    auto s = frame(1);
    s.terminal_identity = b;
    const auto x = m.step(s);
    assert(x.stage == MissionStage::fault && x.hold_zero && x.completed == 0);
    assert(m.step(frame(2)).stage == MissionStage::fault);
  }
  {
    auto m = make_mission();
    auto s = frame(1);
    s.emergency_stop = true;
    assert(m.step(s).stage == MissionStage::fault);
  }
  {
    auto m = make_mission();
    auto s = frame(1);
    s.motion_health_authorized = false;
    assert(m.step(s).stage == MissionStage::paused);
    assert(m.status().hold_zero);
    m.resume(true);
    assert(m.step(frame(2)).stage == MissionStage::verify_hold);
  }
  {
    auto m = make_mission();
    m.step(frame(1));
    auto s = frame(2);
    s.terminal_certificate_valid = false;
    assert(m.step(s).stage == MissionStage::paused);
    assert(m.status().hold_zero);
  }
  {
    auto m = make_mission();
    m.step(frame(1));
    auto s = frame(1.5);
    s.survey = SurveyVerdict::pass;
    s.survey_identity = b;
    s.survey_last_sample_sec = 1.4;
    s.survey_validated_raw_fixed = true;
    assert(m.step(s).stage == MissionStage::fault);
  }
  {
    auto m = make_mission();
    m.step(frame(1));
    auto s = frame(4.0);
    assert(m.step(s).stage == MissionStage::resolved);
    assert(m.records()[0].reason == "RAW_GNSS_TIMEOUT");
    assert(m.status().completed == 0);
  }
  {
    auto m = make_mission();
    m.step(frame(1));
    auto s = frame(1.5);
    s.survey = SurveyVerdict::fail;
    s.survey_identity = a;
    s.survey_last_sample_sec = 1.4;
    s.survey_validated_raw_fixed = true;
    s.survey_radial_error_m = .05;
    assert(m.step(s).stage == MissionStage::resolved);
    assert(m.records()[0].result == MarkingResult::failed);
  }
  {
    // The native raw GNSS gate can report final FAIL because no valid RTK
    // measurement exists. This resolves FAILED, not a spurious fault/spray.
    auto m = make_mission();
    m.step(frame(1));
    auto s = frame(2);
    s.survey = SurveyVerdict::fail;
    s.survey_identity = a;
    assert(m.step(s).stage == MissionStage::resolved);
    assert(m.records()[0].result == MarkingResult::failed);
  }
  {
    // A new survey PASS delivered after the measurement deadline is rejected.
    auto m = make_mission();
    m.step(frame(1));
    auto s = frame(4.1);
    s.survey = SurveyVerdict::pass;
    s.survey_identity = a;
    s.survey_validated_raw_fixed = true;
    s.survey_radial_error_m = .010;
    s.survey_last_sample_sec = 4.0;
    assert(m.step(s).stage == MissionStage::resolved);
    assert(m.records()[0].result == MarkingResult::failed);
  }
  {
    auto m = make_mission();
    m.step(frame(1));
    auto s = frame(1.5);
    s.survey = SurveyVerdict::pass;
    s.survey_identity = a;
    s.survey_last_sample_sec = 1.4;
    s.survey_validated_raw_fixed = true;
    s.survey_radial_error_m = .04;
    assert(m.step(s).stage == MissionStage::resolved);
    assert(m.records()[0].result == MarkingResult::failed);
  }
  {
    auto m = make_mission();
    m.step(frame(1));
    auto s = frame(1.5);
    s.survey = SurveyVerdict::pass;
    s.survey_identity = a;
    s.survey_last_sample_sec = 1.4;
    s.survey_validated_raw_fixed = true;
    s.survey_radial_error_m = .01;
    m.step(s);
    m.step(frame(4.0));
    s = frame(4.1);
    s.spray_result = SprayVerdict::success;
    s.spray_identity = b;
    s.spray_received_sec = 4.1;
    assert(m.step(s).stage == MissionStage::fault);
  }
  {
    auto m = make_mission();
    m.step(frame(1));
    auto s = frame(1.5);
    s.survey = SurveyVerdict::pass;
    s.survey_identity = a;
    s.survey_last_sample_sec = 1.4;
    s.survey_validated_raw_fixed = true;
    m.step(s);
    m.step(frame(4));
    assert(m.step(frame(10)).stage == MissionStage::resolved);
    assert(m.records()[0].reason == "SPRAY_ACK_TIMEOUT");
  }
  {
    auto m = make_mission(MissionMode::manual);
    m.step(frame(1));
    auto s = frame(1.5);
    s.survey = SurveyVerdict::fail;
    s.survey_identity = a;
    s.survey_last_sample_sec = 1.4;
    s.survey_validated_raw_fixed = true;
    s.survey_radial_error_m = .05;
    m.step(s);
    assert(m.step(frame(1.6)).stage == MissionStage::await_next);
    assert(m.status().hold_zero);
    m.next();
    assert(m.stage() == MissionStage::approaching && m.current_marking() == 1);
  }
  {
    auto m = make_mission();
    auto s = frame(1);
    s.motion_health_authorized = false;
    assert(m.step(s).stage == MissionStage::paused);
    s = frame(2);
    s.emergency_stop = true;
    assert(m.step(s).stage == MissionStage::fault);
    bool rejected = false;
    try { m.resume(true); } catch (const std::logic_error &) { rejected = true; }
    assert(rejected);
  }
  {
    auto m = make_mission();
    m.step(frame(1));
    assert(m.step(frame(0.5)).stage == MissionStage::fault);
  }
  {
    auto m = make_mission();
    auto s = frame(1);
    s.measured_forward_speed_mps = -.1;
    assert(m.step(s).stage == MissionStage::fault);
  }
  {
    auto m = make_mission();
    bool caught = false;
    m.pause();
    try { m.resume(false); } catch (const std::logic_error &) { caught = true; }
    assert(caught);
    caught = false;
    try { m.next(); } catch (const std::logic_error &) { caught = true; }
    assert(caught);
    caught = false;
    try { m.load({{1,0,"P0001",a}, {2,1,"P0002",a}}, MissionMode::automatic); }
    catch (const std::invalid_argument &) { caught = true; }
    assert(caught);
  }
  return 0;
}
