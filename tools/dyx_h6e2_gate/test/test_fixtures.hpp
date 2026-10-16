#pragma once
// Synthetic OFFLINE JSON shaped after source APIs, not captured live evidence.
#include "dyx_backend_cpp17_legacy_readonly/legacy_readonly.hpp"
#include <string>
using namespace dyx::h6e2;
inline const std::string mission_fixture=R"({"success":true,"mission":{"state":"EMPTY","loaded":false,"ready":false,"accepted_for_start":false,"mission_enable":false,"emergency_stop":true,"rtk_fixed":false,"marking_active":false,"resume_available":false,"total_points":0,"completed_points":0,"failed_points":0,"skipped_points":0,"remaining_points":0,"point_status":[],"point_results":{}},"report":{}})";
inline const std::string telemetry_fixture=R"({"generated_at":"2026-10-16T00:00:00Z","revision":1,"backend":{},"ros":{},"vehicle":{},"position":{},"gps":{},"estimator":{},"rtk":{},"battery":{},"accuracy":{},"safety":{"emergency_stop":true,"mission_enable":false},"obstacle":{},"mission":{},"emergency_stop":true,"mission_enable":false,"rtk_fixed":false,"armed":false,"connected":false,"battery_mission_lockout":true,"marking_active":false,"mission_state":"EMPTY","gps_fix":0})";
inline TrustedSnapshot snapshot(const std::string& body,std::uint64_t sequence=1,double received=100.0,std::uint64_t epoch=10){return {body,epoch,sequence,received,true};}
inline ReadRequest request(const std::string& path,double time=100.1){return {"GET",path,"",{true,true},time};}
