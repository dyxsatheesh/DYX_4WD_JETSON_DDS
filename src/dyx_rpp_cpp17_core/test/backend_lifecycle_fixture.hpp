#pragma once
#include "dyx_rpp_cpp17_core/backend_request_lifecycle.hpp"
#include <cassert>
#include <cstdint>
#include <stdexcept>
#include <string>
using namespace dyx::rpp;

inline OfflineApiEnvelope offline_get(std::uint64_t id, const char* path,
                                      double received=1.0) {
  return {id, "GET", path, "", "", received};
}
inline OfflineApiEnvelope offline_post(std::uint64_t id, const char* path,
                                       double received=1.0,
                                       std::uint64_t sequence=1) {
  return {id, "POST", path, "application/json; charset=UTF-8",
      "{\"mission_run_id\":\"RUN_LIFECYCLE_0001\",\"sequence\":" +
      std::to_string(sequence) + "}", received};
}
