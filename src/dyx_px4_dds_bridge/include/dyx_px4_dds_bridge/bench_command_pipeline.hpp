#pragma once
// DYX dry-run rover command pipeline. This produces only diagnostics;
// cannot confer authority to publish PX4 setpoints.
#include "dyx_px4_dds_bridge/wheel_lift_gate.hpp"
#include <cmath>
namespace dyx::bridge {
struct BenchCommandInput {
  bool motion_request_seen{false};
  bool mission_preview_fresh{false};
  bool mission_stop_requested{true};
  bool mission_motion_authorized{false};
  bool heading_fresh{false};
  bool yaw_valid{false};
  bool command_fresh{false};
  bool heartbeat_fresh{false};
  bool output_writer_quiet{false};
  double forward_mps{0.0};
  double yaw_rate_radps{0.0};
};
struct BenchCommandOutput {
  bool candidate_valid{false};
  bool physical_publish_authorized{false};
  double staged_speed_mps{0.0};
  double staged_yaw_rate_radps{0.0};
  const char* reason{"NOT_EVALUATED"};
};
inline BenchCommandOutput evaluate_bench_command(const BenchCommandInput& e) noexcept {
  if (!e.motion_request_seen) return {false,false,0,0,"NO_MOTION_REQUEST"};
  if (!e.mission_preview_fresh) return {false,false,0,0,"MISSION_PREVIEW_STALE"};
  if (e.mission_stop_requested) return {false,false,0,0,"MISSION_STOP"};
  if (!e.mission_motion_authorized) return {false,false,0,0,"MISSION_NOT_AUTHORIZED"};
  if (!e.heading_fresh || !e.yaw_valid) return {false,false,0,0,"HEADING_INVALID"};
  if (!e.command_fresh) return {false,false,0,0,"COMMAND_STALE"};
  if (!e.heartbeat_fresh) return {false,false,0,0,"HEARTBEAT_STALE"};
  if (!e.output_writer_quiet) return {false,false,0,0,"OTHER_WRITER"};
  if (!std::isfinite(e.forward_mps) || !std::isfinite(e.yaw_rate_radps))
    return {false,false,0,0,"NONFINITE_COMMAND"};
  if (std::fabs(e.forward_mps)>1.0 || std::fabs(e.yaw_rate_radps)>0.18)
    return {false,false,0,0,"RPP_COMMAND_LIMIT"};
  return {true,false,e.forward_mps,e.yaw_rate_radps,"CANDIDATE_VALID_PHYSICAL_LOCKED"};
}
} // namespace dyx::bridge
