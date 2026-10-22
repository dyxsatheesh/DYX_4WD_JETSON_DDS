#include "dyx_rpp_cpp17_core/mission_sequence.hpp"
#include <stdexcept>
#include <iostream>
#include <string>
using namespace dyx::rpp;
static int count=0;
static void check(bool ok,const char* why){if(!ok) throw std::runtime_error(why);++count;}
static std::vector<MarkingGoal> goals(){return {{0,0,"P0001","runA:path0:P0001"},{1,1,"P0002","runA:path1:P0002"}};}
static MissionSequenceInput input(double t){ MissionSequenceInput x{};x.monotonic_time_sec=t;x.motion_health_authorized=true;x.emergency_stop=false;return x;}
static void terminal(MissionSequenceInput& x,const std::string& id){x.terminal_present=true;x.terminal_identity=id;x.terminal_certificate_valid=true;x.terminal_zero_latched=true;x.terminal_stationary=true;x.terminal_radial_error_m=.01;x.measured_forward_speed_mps=0;}
static void survey(MissionSequenceInput& x,const std::string& id,double sample){x.survey=SurveyVerdict::pass;x.survey_identity=id;x.survey_validated_raw_fixed=true;x.survey_radial_error_m=.012;x.survey_last_sample_sec=sample;}
int main(){
  { // Two-point automatic mission: terminal -> post-stop survey -> hold -> spray ACK -> next -> finished.
    MissionSequencer m; m.load(goals(),MissionMode::automatic);
    check(m.stage()==MissionStage::ready,"auto ready");m.start(true);
    auto a=input(1.0);terminal(a,"runA:path0:P0001");
    check(m.step(a).event==MissionEvent::stop_verified,"P1 stop");
    a=input(1.1);terminal(a,"runA:path0:P0001");survey(a,"runA:path0:P0001",1.1);
    check(m.step(a).stage==MissionStage::verify_hold,"P1 survey accepted");
    a=input(4.1);terminal(a,"runA:path0:P0001");
    check(m.step(a).request_spray,"P1 spray intent");
    a=input(4.2);terminal(a,"runA:path0:P0001");a.spray_result=SprayVerdict::success;a.spray_identity="runA:path0:P0001";a.spray_received_sec=4.2;
    check(m.step(a).stage==MissionStage::resolved,"P1 resolved");
    a=input(4.3);check(m.step(a).event==MissionEvent::advanced,"P2 advance");
    a=input(5.0);terminal(a,"runA:path1:P0002");check(m.step(a).event==MissionEvent::stop_verified,"P2 stop");
    a=input(5.1);terminal(a,"runA:path1:P0002");survey(a,"runA:path1:P0002",5.1);m.step(a);
    a=input(8.1);terminal(a,"runA:path1:P0002");check(m.step(a).request_spray,"P2 spray intent");
    a=input(8.2);terminal(a,"runA:path1:P0002");a.spray_result=SprayVerdict::success;a.spray_identity="runA:path1:P0002";a.spray_received_sec=8.2;
    check(m.step(a).stage==MissionStage::resolved,"P2 resolved");
    a=input(8.3);auto o=m.step(a);check(o.stage==MissionStage::finished,"finished");check(o.completed==2&&o.failed==0&&o.skipped==0,"counts accurate");
  }
  { // Manual mode: explicit operator NEXT only.
    MissionSequencer m;m.load(goals(),MissionMode::manual);m.start(true);
    auto a=input(1.0);terminal(a,"runA:path0:P0001");m.step(a);
    a=input(1.1);terminal(a,"runA:path0:P0001");survey(a,"runA:path0:P0001",1.1);m.step(a);
    a=input(4.1);terminal(a,"runA:path0:P0001");m.step(a);
    a=input(4.2);terminal(a,"runA:path0:P0001");a.spray_result=SprayVerdict::success;a.spray_identity="runA:path0:P0001";a.spray_received_sec=4.2;m.step(a);
    a=input(4.3);check(m.step(a).stage==MissionStage::await_next,"manual await next");
    m.next();check(m.stage()==MissionStage::approaching,"manual explicit next");m.skip();check(m.stage()==MissionStage::resolved,"skip resolved");
    a=input(4.4);check(m.step(a).stage==MissionStage::await_next,"manual await final");m.next();check(m.stage()==MissionStage::finished,"manual finished");
    check(m.status().completed==1&&m.status().skipped==1,"manual count");
  }
  { // Pause/resume requires explicit external gate; STOP terminal.
    MissionSequencer m;m.load(goals(),MissionMode::automatic);m.start(true);m.pause();
    check(m.stage()==MissionStage::paused,"paused");
    bool denied=false;try{m.resume(false);}catch(const std::logic_error&){denied=true;}
    check(denied&&m.stage()==MissionStage::paused,"unsafe resume refused");
    m.resume(true);check(m.stage()==MissionStage::approaching,"explicit resume");
    m.stop();check(m.stage()==MissionStage::stopped,"stop latched");
    denied=false;try{m.resume(true);}catch(const std::logic_error&){denied=true;}
    check(denied,"stop cannot resume");
  }
  { // E-stop, identity mismatch and timeout behavior.
    MissionSequencer m;m.load(goals(),MissionMode::automatic);m.start(true);
    auto a=input(1);a.emergency_stop=true;check(m.step(a).stage==MissionStage::fault,"estop faults");
    bool denied=false;try{m.start(true);}catch(const std::logic_error&){denied=true;}
    check(denied,"fault cannot restart");
  }
  {
    MissionSequencer m;m.load(goals(),MissionMode::automatic);m.start(true);
    auto a=input(1);terminal(a,"incorrect-goal");check(m.step(a).stage==MissionStage::fault,"wrong identity fault");
  }
  {
    MissionSequencer m;m.load(goals(),MissionMode::automatic);m.start(true);
    auto a=input(1);terminal(a,"runA:path0:P0001");m.step(a);
    a=input(4.01);terminal(a,"runA:path0:P0001");
    auto o=m.step(a);check(o.stage==MissionStage::resolved&&o.failed==1,"raw GNSS timeout fails point");
    check(!o.request_spray,"no spray after raw GNSS timeout");
  }
  std::cout<<"PASS: "<<count<<" mission lifecycle assertions; test only, no actuator outputs\n";
}
