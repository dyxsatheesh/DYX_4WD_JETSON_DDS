#pragma once
// Phase 2H3C: PX4 VehicleCommandAck observation ONLY (C++17, no ROS dependency).
// PX4's raw ACK topic does NOT uniquely correlate a command=187 request.
// This monitor deliberately exposes NO command publishing or actuation API.
#include "dyx_rpp_cpp17_core/dds_actuator_transport.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace dyx::rpp {

struct DdsMonitorConfig {
  double observation_fresh_sec{2.0};
  std::uint32_t actuator_command{187};
  void validate() const {
    if (!std::isfinite(observation_fresh_sec) || observation_fresh_sec <= 0.0 ||
        observation_fresh_sec > 30.0 || actuator_command != 187)
      throw std::invalid_argument("invalid DDS monitor config");
  }
};

enum class DdsObservationResult {
  observed, unrelated, invalid_clock, invalid_payload, duplicate_or_reordered
};

struct DdsMonitorStatus {
  std::size_t observed_actuator_acks{0};
  std::size_t accepted_actuator_acks{0};
  std::size_t refused_actuator_acks{0};
  std::size_t unrelated_acks{0};
  std::size_t rejected_observations{0};
  bool last_ack_fresh{false};
  bool last_ack_was_accepted{false};
  bool integrity_fault_latched{false};
  // Invariant: a raw DDS ACK cannot authorize an actuator, motion, success,
  // an individually correlated receipt, or physical-OFF proof.
  bool command_dispatch_permitted{false};
  bool unique_receipt_proven{false};
  bool physical_off_proven{false};
  bool actuator_success_proven{false};
};

class NativeDdsActuatorMonitor final {
 public:
  explicit NativeDdsActuatorMonitor(DdsMonitorConfig cfg={}) : cfg_(cfg) {
    cfg_.validate();
  }

  DdsObservationResult observe(const ObservedPx4Ack &ack, double received_at_sec) {
    if (!std::isfinite(received_at_sec) || received_at_sec < 0.0 ||
        (last_host_time_ && received_at_sec < *last_host_time_)) {
      integrity_fault_latched_=true;
      ++rejected_;
      return DdsObservationResult::invalid_clock;
    }
    last_host_time_=received_at_sec;
    if (ack.command!=cfg_.actuator_command) {
      ++unrelated_;
      return DdsObservationResult::unrelated;
    }
    if (ack.px4_timestamp_us==0 || ack.result>9 || ack.target_system==0 ||
        ack.target_component==0) {
      ++rejected_;
      return DdsObservationResult::invalid_payload;
    }
    // Use PX4 microsecond timestamps only for ordering within the same
    // PX4 boot. Reboot/reset must create a NEW monitor instance; never treat
    // a backwards timestamp as positive confirmation.
    if (last_px4_ack_stamp_ && ack.px4_timestamp_us<=*last_px4_ack_stamp_) {
      ++rejected_;
      return DdsObservationResult::duplicate_or_reordered;
    }
    last_px4_ack_stamp_=ack.px4_timestamp_us;
    last_tracked_rx_=received_at_sec;
    last_accepted_=ack.result==0;
    ++observed_;
    if (last_accepted_) ++accepted_;
    else ++refused_;
    return DdsObservationResult::observed;
  }

  DdsMonitorStatus status(double now_sec) const {
    DdsMonitorStatus s;
    s.observed_actuator_acks=observed_;
    s.accepted_actuator_acks=accepted_;
    s.refused_actuator_acks=refused_;
    s.unrelated_acks=unrelated_;
    s.rejected_observations=rejected_;
    s.integrity_fault_latched=integrity_fault_latched_;
    s.last_ack_was_accepted=last_accepted_;
    s.last_ack_fresh=last_tracked_rx_ && std::isfinite(now_sec) &&
        now_sec>=*last_tracked_rx_ &&
        now_sec-*last_tracked_rx_<=cfg_.observation_fresh_sec &&
        !integrity_fault_latched_;
    return s;
  }

 private:
  DdsMonitorConfig cfg_{};
  std::optional<double> last_host_time_{},last_tracked_rx_{};
  std::optional<std::uint64_t> last_px4_ack_stamp_{};
  std::size_t observed_{0},accepted_{0},refused_{0},unrelated_{0},rejected_{0};
  bool last_accepted_{false},integrity_fault_latched_{false};
};
} // namespace dyx::rpp
