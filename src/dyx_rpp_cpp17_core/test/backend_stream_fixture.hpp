#pragma once
#include "backend_http_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_live_stream.hpp"
#include <cassert>
#include <string>
using namespace dyx::rpp;

inline void stream_setup(NativeMissionCommandGateway& g,
                         NativeBackendLiveStatus& live, double now = 100.0) {
  http_load(g);
  backend_live_feed(g, now);
  assert(live.observe_tick(backend_live_tick(g,now)));
  assert(live.observe_satellites({now,0x1234,18}));
}
inline BackendTrustedHttpRequest stream_read(double now) {
  return http_read(kBackendLiveStatusPath,now);
}
