#include "dyx_rpp_cpp17_core/spray_mission_adapter.hpp"
#include <cassert>
#include <iostream>
using namespace dyx::rpp;
int main(){
 MissionSequencer mission;
 const std::string id="RUN:alpha|PATH:digest|GOAL:P0001";
 mission.load({MarkingGoal{0,0,"P0001",id}},MissionMode::automatic);
 mission.start(true);
 MissionSequenceInput m;
 m.monotonic_time_sec=1;m.motion_health_authorized=true;m.emergency_stop=false;
 m.terminal_present=true;m.terminal_identity=id;m.terminal_certificate_valid=true;
 m.terminal_zero_latched=true;m.terminal_stationary=true;
 m.terminal_radial_error_m=0.010;m.measured_forward_speed_mps=0.0;
 assert(mission.step(m).stage==MissionStage::verify_hold);
 m.monotonic_time_sec=1.4;m.survey=SurveyVerdict::pass;m.survey_identity=id;
 m.survey_validated_raw_fixed=true;m.survey_radial_error_m=0.015;m.survey_last_sample_sec=1.3;
 assert(mission.step(m).stage==MissionStage::verify_hold);
 m.monotonic_time_sec=4.1;auto request=mission.step(m);
 assert(request.request_spray && request.stage==MissionStage::spray_ack);
 SprayOutput untrusted;untrusted.completion=SprayCompletion::completed;
 untrusted.completed_identity=id;untrusted.completion_time_sec=4.2;
 m.monotonic_time_sec=7.5;
 assert(!bind_spray_release_to_mission(untrusted,m)); // no release ACK
 untrusted.release_acknowledged=true;untrusted.completed_identity="OTHER_GOAL";
 assert(!bind_spray_release_to_mission(untrusted,m));
 untrusted.completed_identity=id;
 assert(bind_spray_release_to_mission(untrusted,m));
 assert(mission.step(m).stage==MissionStage::resolved);
 assert(mission.step(m).stage==MissionStage::finished);
 assert(mission.records()[0].result==MarkingResult::completed);
 std::cout<<"Mission completion requires exact identity and confirmed spray RELEASE";
}
