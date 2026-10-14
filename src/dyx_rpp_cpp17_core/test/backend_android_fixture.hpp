#pragma once
// All test inputs are synthetic. There is NO Android, network or PX4 runtime.
#include "backend_stream_fixture.hpp"
#include "dyx_rpp_cpp17_core/backend_android_telemetry_contract.hpp"
#include <cassert>
#include <string>
using namespace dyx::rpp;

inline void assert_android_safe(const BackendAndroidReplicaView& v) {
  assert(!v.mission_commands_enabled);
  assert(!v.network_io_performed);
  assert(!v.hardware_command_emitted);
}
inline void assert_android_disconnected(const BackendAndroidReplicaView& v) {
  assert_android_safe(v);
  assert(!v.connected && !v.snapshot_fresh && !v.rtk_fixed);
  assert(v.requires_resync && v.status_json.empty());
}
