#include "dyx_rpp_cpp17_core/raw_survey_accuracy.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace dyx::rpp;

static RawGnssFix f(double t, double lat, double lon, int type=6) {
  return {t,lat,lon,type,18,0.015};
}
int main() {
  const SurveyTarget p{"P0001", 0, 12.0, 77.0};
  const SurveyTarget before{"P0000", -1, 11.99998, 77.0};
  const std::vector<RawGnssFix> near{
    f(10.20,12.0,77.00000011),
    f(10.40,12.0,77.00000013),
    f(10.60,12.0,77.00000012)
  };
  const auto d=evaluate_raw_survey("RUN:A|P:1",10.0,10.65,p,before,near);
  assert(d.verdict==RawSurveyVerdict::pass);
  assert(d.validated_raw_fixed && d.radial_error_m);
  assert(*d.radial_error_m > 0.010 && *d.radial_error_m < 0.020);
  assert(d.sample_count==3 && d.trimmed_count==0);
  assert(d.sample_span_sec>0.39 && d.sample_span_sec<0.41);
  assert(d.last_sample_monotonic_sec && *d.last_sample_monotonic_sec==10.60);
  assert(d.cross_track_error_m && *d.cross_track_error_m>0.0);
  assert(d.along_track_error_m && std::abs(*d.along_track_error_m)<0.002);
  assert(d.bearing_source=="SURVEYED_SEGMENT");
  assert(d.terminal_identity=="RUN:A|P:1");
  const auto first=evaluate_raw_survey("A",10.0,10.65,p,std::nullopt,near);
  assert(first.passed() && first.bearing_source.empty()); // P1 needs no bearing for radial accuracy
  assert(!first.cross_track_error_m);
  const auto early=evaluate_raw_survey("A",10.0,10.25,p,std::nullopt,near);
  assert(early.verdict==RawSurveyVerdict::waiting);
  assert(early.reason=="COLLECTING_STATIONARY_RAW_GNSS");
  const auto far=evaluate_raw_survey("A",10.0,10.65,p,std::nullopt,
    {f(10.2,12,77.0000012),f(10.4,12,77.0000012),f(10.6,12,77.0000012)});
  assert(far.verdict==RawSurveyVerdict::fail);
  assert(far.reason=="RAW_GNSS_RADIAL_EXCEEDS_TOLERANCE");
  assert(far.validated_raw_fixed && far.radial_error_m && *far.radial_error_m>0.10);
  // WGS84 inverse should agree with known equatorial meridional interval.
  const double d1=survey_detail::wgs84_inverse_distance_m(0,0,0.000001,0);
  assert(std::abs(d1 - 0.110574) < 0.0003);
  std::cout<<"raw survey valid and radial-pass/fail cases OK\n";
}
