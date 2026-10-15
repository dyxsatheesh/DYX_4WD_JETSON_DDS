#pragma once
// DYX Autonomous Phase 2H5T -- OFFLINE host supervision laboratory ONLY.
// NO real process, timer, socket, TLS, systemd service, authenticated ingress,
// DDS/PX4 publisher, physical interlock, brake or actuator exists here.
// All "independent" input is a contract for a future privileged host adapter:
// this type cannot verify the source of a boolean, principal or clock.

#include "dyx_rpp_cpp17_core/backend_recovery_trusted_ingress_lab.hpp"
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <utility>

namespace dyx::rpp {

enum class RecoveryHostSupervisorLabState {
  initial_closed,
  recovery_only_admitted,
  permanently_closed
};

// These are test-only declarations of the intended attachment configuration,
// not actual inspection or attestation of the host or hardware.
struct RecoveryHostSupervisorLabConfig {
  double max_heartbeat_gap_sec{5.0};
  bool offline_no_listener_attached{true};
  bool offline_no_dds_publisher_attached{true};
  bool offline_no_actuator_attached{true};
};

struct RecoveryHostStartupEvidenceLab {
  bool independent_recovery_review_approved{false};
  bool independent_outputs_quiesced{false};
  std::string verified_operator_session_id{};
  double trusted_monotonic_now_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct RecoveryHostSupervisorLabSnapshot {
  RecoveryHostSupervisorLabState state{
      RecoveryHostSupervisorLabState::initial_closed};
  bool accepts_recovery_requests{false};
  bool can_issue_motion{false};
  bool can_upload_or_load{false};
  bool network_io_performed{false};
  bool hardware_command_emitted{false};
  bool physical_stop_confirmed{false};
  double last_host_clock_sec{0.0};
  double last_heartbeat_sec{0.0};
};

// A single *offline* simulated host instance. Boot creates the H5S ingress
// before admission. No reference to any inner layer or actuator escapes.
// Lock ordering: H5T mutex -> H5S -> H5R -> H5Q. No callbacks while locked.
class NativeOfflineRecoveryHostSupervisorLab final {
 public:
  NativeOfflineRecoveryHostSupervisorLab(
      std::string sqlite_file,
      const DraftQuarantinedReviewLabRequest& independently_verified_review,
      RecoveryHostSupervisorLabConfig config = {})
      : reviewed_session_(
            independently_verified_review.current_identity.operator_session_id),
        config_(config),
        ingress_(std::move(sqlite_file), independently_verified_review) {}

  NativeOfflineRecoveryHostSupervisorLab(
      const NativeOfflineRecoveryHostSupervisorLab&) = delete;
  NativeOfflineRecoveryHostSupervisorLab& operator=(
      const NativeOfflineRecoveryHostSupervisorLab&) = delete;
  NativeOfflineRecoveryHostSupervisorLab(
      NativeOfflineRecoveryHostSupervisorLab&&) = delete;
  NativeOfflineRecoveryHostSupervisorLab& operator=(
      NativeOfflineRecoveryHostSupervisorLab&&) = delete;

  // All checks precede admission. A failed attempt permanently seals this
  // host instance rather than retrying against partially initialized state.
  bool start_recovery_only(const RecoveryHostStartupEvidenceLab& evidence) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (state_ != RecoveryHostSupervisorLabState::initial_closed)
      return false;
    if (!valid_config() ||
        !evidence.independent_recovery_review_approved ||
        !evidence.independent_outputs_quiesced ||
        reviewed_session_.empty() ||
        evidence.verified_operator_session_id != reviewed_session_ ||
        !valid_time(evidence.trusted_monotonic_now_sec)) {
      close_locked();
      return false;
    }
    try {
      if (!ingress_.admit_recovery_only()) {
        close_locked();
        return false;
      }
      state_ = RecoveryHostSupervisorLabState::recovery_only_admitted;
      boot_sec_ = evidence.trusted_monotonic_now_sec;
      last_clock_sec_ = boot_sec_;
      last_heartbeat_sec_ = boot_sec_;
      return true;
    } catch (...) {
      close_locked();
      return false;
    }
  }

  // A future privileged scheduler MUST call this; there is deliberately no
  // timer thread. A late heartbeat cannot revive a timed-out host instance.
  bool observe_heartbeat(double trusted_host_now_sec) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!check_clock_and_watchdog_locked(trusted_host_now_sec)) return false;
    last_heartbeat_sec_ = trusted_host_now_sec;
    return true;
  }

  BackendHttpWireResponse dispatch(
      const BackendTrustedUploadHttpRequest& raw_client_request,
      const RecoveryTrustedIngressEvidenceLab& independent_evidence,
      double trusted_host_now_sec) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!check_clock_and_watchdog_locked(trusted_host_now_sec))
      return blocked();
    // Independent ingress receive time must agree with the independent host
    // time and must not predate this host incarnation. Otherwise fail closed.
    const double received = independent_evidence.trusted_monotonic_received_sec;
    if (!valid_time(received) || received < boot_sec_ ||
        received > trusted_host_now_sec ||
        trusted_host_now_sec - received > config_.max_heartbeat_gap_sec) {
      close_locked();
      return blocked();
    }
    try {
      auto result = ingress_.dispatch(raw_client_request, independent_evidence);
      const auto downstream = ingress_.snapshot();
      if (downstream.closed_permanently ||
          result.physical_command_emitted || result.network_io_performed ||
          result.body.size() > 16384) {
        close_locked();
        return blocked();
      }
      return result;
    } catch (...) {
      close_locked();
      return blocked();
    }
  }

  void shutdown() noexcept {
    std::lock_guard<std::mutex> lk(mutex_);
    close_locked();
  }

  RecoveryHostSupervisorLabSnapshot snapshot() const {
    std::lock_guard<std::mutex> lk(mutex_);
    RecoveryHostSupervisorLabSnapshot r;
    r.state = state_;
    r.accepts_recovery_requests =
        state_ == RecoveryHostSupervisorLabState::recovery_only_admitted;
    r.last_host_clock_sec = last_clock_sec_;
    r.last_heartbeat_sec = last_heartbeat_sec_;
    return r;
  }

 private:
  static bool valid_time(double sec) {
    return std::isfinite(sec) && sec >= 0.0;
  }
  bool valid_config() const {
    return std::isfinite(config_.max_heartbeat_gap_sec) &&
        config_.max_heartbeat_gap_sec >= 0.1 &&
        config_.max_heartbeat_gap_sec <= 60.0 &&
        config_.offline_no_listener_attached &&
        config_.offline_no_dds_publisher_attached &&
        config_.offline_no_actuator_attached;
  }
  bool check_clock_and_watchdog_locked(double now) {
    if (state_ != RecoveryHostSupervisorLabState::recovery_only_admitted)
      return false;
    if (!valid_time(now) || now < last_clock_sec_ ||
        now - last_heartbeat_sec_ > config_.max_heartbeat_gap_sec) {
      close_locked();
      return false;
    }
    last_clock_sec_ = now;
    return true;
  }
  static BackendHttpWireResponse blocked() {
    return {423, http_json::error_json("RECOVERY_HOST_NOT_ACCEPTING")};
  }
  void close_locked() noexcept {
    state_ = RecoveryHostSupervisorLabState::permanently_closed;
    try { ingress_.shutdown(); } catch (...) { /* State remains sealed. */ }
  }

  mutable std::mutex mutex_{};
  const std::string reviewed_session_;
  const RecoveryHostSupervisorLabConfig config_;
  NativeOfflineRecoveryTrustedIngressLab ingress_;
  RecoveryHostSupervisorLabState state_{RecoveryHostSupervisorLabState::initial_closed};
  double boot_sec_{0.0};
  double last_clock_sec_{0.0};
  double last_heartbeat_sec_{0.0};
};

} // namespace dyx::rpp
