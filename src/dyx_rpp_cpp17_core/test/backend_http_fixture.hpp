#pragma once
// Fully synthetic HTTP wire tests: no network, hardware, TLS or ROS 2.
#include "backend_live_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_http_wire.hpp"
#include <cassert>
#include <string>

inline BackendTrustedHttpRequest http_read(const char* path, double now) {
  BackendTrustedHttpRequest r;
  r.path = path;
  r.host_received_sec = now;
  r.principal = {true, true, false, "SESSION_HTTP_READER"};
  return r;
}
inline BackendTrustedHttpRequest http_post(const char* path,
                                            std::uint64_t sequence,
                                            double now) {
  BackendTrustedHttpRequest r;
  r.method = "POST";
  r.path = path;
  r.host_received_sec = now;
  r.principal = {true, true, true, "SESSION_HTTP_OPERATOR"};
  r.body = "{\"mission_run_id\":\"RUN_HTTP_0001\",\"sequence\":" +
           std::to_string(sequence) + "}";
  return r;
}
inline void http_load(NativeMissionCommandGateway& g) {
  assert(g.load("RUN_HTTP_0001", "SESSION_HTTP_OPERATOR",
      {{0,0,"P0001","RUN:HTTP:P0001"},{100,1,"P0002","RUN:HTTP:P0002"}},
      MissionMode::automatic, true, true, true) == ClientCommandStatus::accepted);
}
