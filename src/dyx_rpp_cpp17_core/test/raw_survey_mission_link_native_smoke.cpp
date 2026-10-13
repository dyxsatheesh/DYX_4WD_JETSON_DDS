#include "dyx_rpp_cpp17_core/raw_survey_mission_adapter.hpp"
#include <cassert>
#include <stdexcept>
#include <iostream>
using namespace dyx::rpp;
static RawGnssFix f(double t){return {t,12,77,6,18,.010};}
int main(){
 const SurveyTarget target{"P0001",0,12,77};
 auto pass=evaluate_raw_survey("RUN:A|GOAL:P1",0,.65,target,{}, {f(.2),f(.4),f(.6)});
 assert(pass.passed());
 MissionSequenceInput s;
 s.monotonic_time_sec=0;
 s.motion_health_authorized=true;
 s.emergency_stop=false;
 s.terminal_present=true;
 s.terminal_identity="RUN:A|GOAL:P1";
 s.terminal_certificate_valid=true;
 s.terminal_zero_latched=true;
 s.terminal_stationary=true;
 s.terminal_radial_error_m=.010;
 s.measured_forward_speed_mps=0.0;
 bool threw=false;
 try {apply_raw_survey_evidence(s,pass,"RUN:B|GOAL:P1");}
 catch(const std::invalid_argument&) {threw=true;}
 assert(threw); // cross-run evidence not silently accepted
 apply_raw_survey_evidence(s,pass,"RUN:A|GOAL:P1");
 assert(s.survey==SurveyVerdict::pass && s.survey_validated_raw_fixed);
 MissionSequencer mission;
 mission.load({{1,0,"P0001","RUN:A|GOAL:P1"}},MissionMode::automatic);
 mission.start(true);
 auto out=mission.step(s);
 assert(out.stage==MissionStage::verify_hold && out.hold_zero);
 s.monotonic_time_sec=.65;
 out=mission.step(s);
 assert(out.stage==MissionStage::verify_hold && !out.request_spray);
 s.monotonic_time_sec=3.01;
 out=mission.step(s);
 assert(out.stage==MissionStage::spray_ack && out.request_spray);
 s.monotonic_time_sec=3.11;
 s.spray_result=SprayVerdict::success;
 s.spray_identity=s.terminal_identity;
 s.spray_received_sec=3.10;
 out=mission.step(s);
 assert(out.stage==MissionStage::resolved && out.completed==1);
 s.monotonic_time_sec=3.12;
 out=mission.step(s);
 assert(out.stage==MissionStage::finished && out.completed==1);
 // A waiting survey cannot manufacture a PASS. A forged PASS is rejected.
 MissionSequenceInput t;
 RawSurveyDecision waiting;waiting.terminal_identity="RUN:A|GOAL:P1";
 apply_raw_survey_evidence(t,waiting,"RUN:A|GOAL:P1");
 assert(t.survey==SurveyVerdict::waiting && !t.survey_validated_raw_fixed);
 pass.radial_error_m=1.0;
 threw=false;
 try {apply_raw_survey_evidence(t,pass,"RUN:A|GOAL:P1");}
 catch(const std::invalid_argument&) {threw=true;}
 assert(threw);
 std::cout<<"raw survey to mission sequencer evidence handoff OK\n";
}
