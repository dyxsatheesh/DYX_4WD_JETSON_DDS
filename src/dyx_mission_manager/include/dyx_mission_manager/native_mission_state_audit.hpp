#pragma once
// D1C: passive C++17 mission-stage transition audit.
// Observes only the single ROS mission owner. Never dispatches a command,
// grants a safety lease, clears a STOP latch, or emits hardware output.
#include "dyx_rpp_cpp17_core/mission_sequence.hpp"
#include <cstdint>

namespace dyx::http_backend {
class NativeMissionStateAudit final {
public:
  void observe(dyx::rpp::MissionStage stage, bool mailbox_stop_latched,
               bool owner_safe_stop_requested) noexcept {
    if (!seen_) {
      stage_ = stage;
      seen_ = true;
    } else if (stage != stage_) {
      // Preserve a one-way STOP boundary. A fresh ROS process is required
      // before a new mission may be admitted after mailbox STOP.
      if (mailbox_stop_latched_ && stage != dyx::rpp::MissionStage::stopped &&
          stage != dyx::rpp::MissionStage::fault)
        ++stop_boundary_violations_;
      // A FAULT->moving transition is never an automatic recovery.
      if (stage_ == dyx::rpp::MissionStage::fault &&
          (stage == dyx::rpp::MissionStage::approaching ||
           stage == dyx::rpp::MissionStage::verify_hold ||
           stage == dyx::rpp::MissionStage::spray_ack))
        ++fault_recovery_violations_;
      previous_ = stage_;
      stage_ = stage;
      ++transitions_;
      if (stage == dyx::rpp::MissionStage::paused) ++pause_entries_;
      if (stage == dyx::rpp::MissionStage::stopped) ++stop_entries_;
      if (stage == dyx::rpp::MissionStage::fault) ++fault_entries_;
    }
    mailbox_stop_latched_ |= mailbox_stop_latched;
    owner_safe_stop_requested_ |= owner_safe_stop_requested;
    if (mailbox_stop_latched_ && stage_ != dyx::rpp::MissionStage::stopped &&
        stage_ != dyx::rpp::MissionStage::fault)
      stop_boundary_observed_ = true; // Diagnostic; STOP delivery may be pending.
  }
  bool seen() const noexcept { return seen_; }
  const char* stage_name() const noexcept { return dyx::rpp::mission_stage_name(stage_); }
  const char* previous_stage_name() const noexcept { return dyx::rpp::mission_stage_name(previous_); }
  std::uint64_t transitions() const noexcept { return transitions_; }
  std::uint64_t pause_entries() const noexcept { return pause_entries_; }
  std::uint64_t stop_entries() const noexcept { return stop_entries_; }
  std::uint64_t fault_entries() const noexcept { return fault_entries_; }
  std::uint64_t stop_boundary_violations() const noexcept { return stop_boundary_violations_; }
  std::uint64_t fault_recovery_violations() const noexcept { return fault_recovery_violations_; }
  bool stop_boundary_observed() const noexcept { return stop_boundary_observed_; }
  bool stop_latched() const noexcept { return mailbox_stop_latched_; }
  bool owner_safe_stop_requested() const noexcept { return owner_safe_stop_requested_; }
  bool motion_authorized() const noexcept { return false; }
private:
  bool seen_{false};
  dyx::rpp::MissionStage stage_{dyx::rpp::MissionStage::unloaded};
  dyx::rpp::MissionStage previous_{dyx::rpp::MissionStage::unloaded};
  std::uint64_t transitions_{0}, pause_entries_{0}, stop_entries_{0}, fault_entries_{0};
  std::uint64_t stop_boundary_violations_{0}, fault_recovery_violations_{0};
  bool mailbox_stop_latched_{false}, owner_safe_stop_requested_{false};
  bool stop_boundary_observed_{false};
};
} // namespace dyx::http_backend
