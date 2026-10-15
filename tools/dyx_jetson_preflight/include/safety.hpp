#pragma once
#include <string>
namespace dyx::preflight {
// This process has NO PX4, ROS, DDS, serial or actuator dependency, deliberately.
// This is a software-only observation service, NOT a certified safety controller.
struct InhibitState {
  bool motion_enabled() const noexcept { return false; }
  bool spray_enabled() const noexcept { return false; }
  bool dds_writer_attached() const noexcept { return false; }
  bool physically_stopped_confirmed() const noexcept { return false; }
  std::string json() const {
    return "{\"service\":\"dyx-jetson-preflight\",\"mode\":\"observe_only\","
           "\"motion_enabled\":false,\"spray_enabled\":false,"
           "\"dds_writer_attached\":false,\"physical_stop_confirmed\":false}";
  }
};
} // namespace dyx::preflight
