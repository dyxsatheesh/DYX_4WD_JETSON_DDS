#pragma once
// C++17 offline tests: all mission positions are synthetic local ENU metres.
#include "dyx_rpp_cpp17_core/backend_upload_http_wire.hpp"
#include <cassert>
#include <cstdint>
#include <string>
using namespace dyx::rpp;

inline std::string draft_upload_json(std::uint64_t seq=1, const std::string& points="[[0,0],[2,0]]") {
  return "{\"mission_run_id\":\"RUN_H4P_0001\",\"sequence\":"+
    std::to_string(seq)+",\"marking_points\":"+points+
    ",\"extension_enabled\":false,\"extension_distance_m\":4.0,\"mission_mode\":\"automatic\"}";
}
inline std::string draft_update_json(std::uint64_t seq, std::uint64_t revision,
                                      const std::string& signature,
                                      const std::string& points="[[0,0],[3,0]]") {
  return "{\"expected_signature\":\""+signature+"\",\"marking_points\":"+points+
    ",\"mission_mode\":\"automatic\",\"extension_distance_m\":4.0,"
    "\"sequence\":"+std::to_string(seq)+",\"expected_revision\":"+
    std::to_string(revision)+",\"extension_enabled\":false,"
    "\"mission_run_id\":\"RUN_H4P_0001\"}";
}
inline std::string draft_load_json(std::uint64_t seq, std::uint64_t revision,
                                    const std::string& signature) {
  return "{\"sequence\":"+std::to_string(seq)+
    ",\"expected_revision\":"+std::to_string(revision)+
    ",\"expected_signature\":\""+signature+
    "\",\"mission_run_id\":\"RUN_H4P_0001\"}";
}
inline BackendTrustedUploadHttpRequest draft_http(const std::string& method,
   const std::string& path, const std::string& body, double time_sec=10.0) {
  BackendTrustedUploadHttpRequest r;
  r.http.method=method;
  r.http.path=path;
  r.http.body=body;
  r.http.host_received_sec=time_sec;
  r.http.principal={true,true,true,"SESSION_H4P_OPERATOR"};
  r.trusted_px4_disarmed=true;
  return r;
}
inline BackendTrustedUploadHttpRequest draft_get(double time_sec=10.0) {
  return draft_http("GET", "/api/v1/mission/draft?mission_run_id=RUN_H4P_0001", "", time_sec);
}
inline bool reply_has(const BackendHttpWireResponse& r, const std::string& s) {
  return r.body.find(s)!=std::string::npos;
}
