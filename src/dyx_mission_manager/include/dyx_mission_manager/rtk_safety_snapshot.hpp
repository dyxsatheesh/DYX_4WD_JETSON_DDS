#pragma once
#include "dyx_mission_manager/native_rtk_float_guard.hpp"
#include "dyx_mission_manager/native_rtcm_age_guard.hpp"
#include <cmath>

namespace dyx::http_backend {
struct RtkSafetySnapshot {
 bool raw_fixed{false};
 bool float_over_five_seconds{false};
 bool pause_intent{true};
 bool correction_age_verified{false};
 bool motion_ready{false};
 const char* reason{"GNSS_STALE_OR_FAULT"};
};
inline RtkSafetySnapshot make_rtk_safety_snapshot(
    const NativeRtkFloatGuard& gps,
    const NativeRtcmAgeGuard& age,
    double now,
    bool independent_receiver_age_provenance,
    bool other_safety_clear) {
 RtkSafetySnapshot s;
 s.raw_fixed=gps.fixed(now);
 s.float_over_five_seconds=gps.float_over_five_seconds(now);
 s.pause_intent=gps.pause_requested(now);
 s.correction_age_verified=independent_receiver_age_provenance &&
     age.receiver_corrections_fresh(now);
 s.motion_ready=s.raw_fixed && s.correction_age_verified &&
     !s.pause_intent && other_safety_clear;
 if(!gps.fresh(now))s.reason="GNSS_STALE_OR_FAULT";
 else if(s.float_over_five_seconds)s.reason="RTK_FLOAT_OVER_5_SECONDS";
 else if(!s.raw_fixed)s.reason="RTK_NOT_FIXED";
 else if(!independent_receiver_age_provenance)s.reason="RECEIVER_AGE_PROVENANCE_UNAVAILABLE";
 else if(!s.correction_age_verified)s.reason="RECEIVER_CORRECTIONS_STALE";
 else if(!other_safety_clear)s.reason="OTHER_SAFETY_HOLD";
 else s.reason="DIAGNOSTIC_READY";
 return s;
}
}
