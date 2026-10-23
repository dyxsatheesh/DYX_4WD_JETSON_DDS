#pragma once
// Stateful non-actuating simulated transport and recovery interlock.
// No ROS publishers, PX4 writers, hardware commands or positive authority.
#include "dyx_px4_dds_bridge/bench_command_pipeline.hpp"
#include <cmath>
#include <cstdint>

namespace dyx::bridge {
struct SimulatedCommandReceipt {
  bool candidate_accepted{false};
  bool latched_stop{true};
  bool physical_publish_authorized{false};
  double simulated_forward_mps{0.0};
  double simulated_yaw_rate_radps{0.0};
  std::uint64_t accepted_count{0};
  const char* reason{"INITIAL_STOP_LOCKED"};
};
class BenchTransportInterlock final {
 public:
  SimulatedCommandReceipt evaluate(const BenchCommandOutput& c,
                                   bool stop_requested,
                                   bool emergency_stop,
                                   bool writer_conflict,
                                   bool heartbeat_fresh,
                                   bool estimator_valid) noexcept {
    if (stop_requested || emergency_stop || writer_conflict ||
        !heartbeat_fresh || !estimator_valid || !c.candidate_valid ||
        !std::isfinite(c.staged_speed_mps) ||
        !std::isfinite(c.staged_yaw_rate_radps) ||
        std::fabs(c.staged_speed_mps) > 1.0 ||
        std::fabs(c.staged_yaw_rate_radps) > 0.18) {
      latched_ = true;
      armed_for_simulation_ = false;
      return {false,true,false,0.0,0.0,accepted_,"STOP_OR_STALE_LATCHED"};
    }
    if (latched_ || !armed_for_simulation_) {
      return {false,true,false,0.0,0.0,accepted_,"EXPLICIT_SIM_RESET_REQUIRED"};
    }
    ++accepted_;
    // Shadow receipt only; does not publish, arm, actuate or bypass PX4.
    return {true,false,false,c.staged_speed_mps,
            c.staged_yaw_rate_radps,accepted_,"SIMULATED_ONLY"};
  }
  // Test-harness reset. Never a motor enable. Must be explicitly invoked and
  // requires zero-motion command and fail-safe inputs. Not exposed via ROS.
  bool reset_simulation(bool operator_reset, bool stop_cleared,
                        bool emergency_stop_cleared, bool writer_quiet,
                        bool heartbeat_fresh, bool estimator_valid,
                        double requested_speed, double requested_yaw_rate) noexcept {
    if (!operator_reset || !stop_cleared || !emergency_stop_cleared ||
        !writer_quiet || !heartbeat_fresh || !estimator_valid ||
        !std::isfinite(requested_speed) || !std::isfinite(requested_yaw_rate) ||
        requested_speed != 0.0 || requested_yaw_rate != 0.0) return false;
    latched_ = false;
    armed_for_simulation_ = true;
    return true;
  }
  bool latched() const noexcept { return latched_; }
 private:
  bool latched_{true};
  bool armed_for_simulation_{false};
  std::uint64_t accepted_{0};
};
} // namespace dyx::bridge
