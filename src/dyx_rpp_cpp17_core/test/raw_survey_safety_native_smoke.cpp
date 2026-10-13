#include "dyx_rpp_cpp17_core/raw_survey_accuracy.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace dyx::rpp;
static RawGnssFix f(double t, double lat=12., double lon=77.,int type=6) {
 return {t,lat,lon,type,16,.020};
}
int main(){
 const SurveyTarget p{"P0001",0,12,77};
 const std::vector<RawGnssFix> good{f(.2),f(.3),f(.5)};
 auto d=evaluate_raw_survey("ID",0,.6,p,{},good);
 assert(d.passed());
 // Pre-stop/old raw GNSS must never authorize marking.
 assert(evaluate_raw_survey("ID",1,1.6,p,{},good).verdict==RawSurveyVerdict::waiting);
 assert(evaluate_raw_survey("ID",1,4.1,p,{},good).verdict==RawSurveyVerdict::fail);
 // RTK FLOAT contaminates whole window even with three FIXED samples.
 const std::vector<RawGnssFix> degraded{f(.2),f(.3),f(.4,12,77,5),f(.5)};
 d=evaluate_raw_survey("ID",0,.6,p,{},degraded);
 assert(d.verdict==RawSurveyVerdict::waiting && d.reason=="GNSS_NOT_RTK_FIXED");
 d=evaluate_raw_survey("ID",0,3.1,p,{},degraded);
 assert(d.verdict==RawSurveyVerdict::fail);
 // No survey target => wait and then fail; EKF position cannot substitute.
 assert(evaluate_raw_survey("ID",0,.7,{}, {},good).verdict==RawSurveyVerdict::waiting);
 assert(evaluate_raw_survey("ID",0,3.1,{}, {},good).verdict==RawSurveyVerdict::fail);
 // Invalid sample inside the stationary window cannot be silently filtered.
 auto invalid=f(.4); invalid.horizontal_accuracy_m=std::numeric_limits<double>::quiet_NaN();
 d=evaluate_raw_survey("ID",0,.6,p,{}, {f(.2),f(.3),invalid,f(.5)});
 assert(d.verdict==RawSurveyVerdict::waiting && d.reason=="INVALID_RAW_GNSS_SAMPLE");
 // Duplicate sensor timestamps do not represent independent GNSS fixes.
 d=evaluate_raw_survey("ID",0,.6,p,{}, {f(.3),f(.3),f(.3)});
 assert(d.verdict==RawSurveyVerdict::waiting && d.reason=="GNSS_DUPLICATE_SAMPLE_TIMESTAMPS");
 // Moving GNSS, including a case where old-end trimming would salvage a
 // stationary subset, must still reject the entire pre-spray window.
 const std::vector<RawGnssFix> moving{
   f(.25,12.000002,77),f(.35,12,77),f(.45,12,77),f(.55,12,77)};
 d=evaluate_raw_survey("ID",0,.6,p,{},moving);
 assert(d.verdict==RawSurveyVerdict::waiting);
 assert(d.reason=="GNSS_WINDOW_NOT_STATIONARY");
 // Delayed sample cannot satisfy gate, and duplicate samples should not
 // upgrade a failed one-shot decision after new data arrives.
 OneShotRawSurveyGate gate;
 gate.start("ID",0);
 assert(gate.step(.2,p,{},good).verdict==RawSurveyVerdict::waiting);
 auto decided=gate.step(.65,p,{}, {f(.2,12,77.000001),f(.4,12,77.000001),f(.6,12,77.000001)});
 assert(decided.verdict==RawSurveyVerdict::fail);
 const auto latched=gate.step(.8,p,{},good);
 assert(latched.verdict==RawSurveyVerdict::fail);
 assert(latched.reason==decided.reason);
 // Bad time cannot be used to leak motion/spray authority.
 assert(evaluate_raw_survey("ID",0,std::numeric_limits<double>::quiet_NaN(),p,{},good).verdict==RawSurveyVerdict::fail);
 assert(evaluate_raw_survey("",0,.6,p,{},good).verdict==RawSurveyVerdict::fail);
 gate.start("ID",0);
 auto reverse_time=gate.step(.7,p,{},good);
 assert(reverse_time.passed());
 assert(gate.step(.6,p,{},good).verdict==RawSurveyVerdict::fail);
 assert(gate.step(.8,p,{},good).verdict==RawSurveyVerdict::fail);
 std::cout<<"raw survey fail-closed and one-shot cases OK\n";
}
