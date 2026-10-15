#pragma once
// DYX Autonomous Phase 2H5R: OFFLINE service-admission lifecycle, not a
// network listener, systemd unit, security principal verifier, DDS or actuator.
// Construction creates the H5N/H5Q quarantined owner before admission can
// open. A service adapter must independently authenticate all trusted input.
#include "dyx_rpp_cpp17_core/backend_recovery_startup_owner_lab.hpp"
#include <exception>
#include <mutex>
#include <string>
#include <utility>

namespace dyx::rpp {

enum class RecoveryServiceLifecycleLabState {
  admission_closed,
  recovery_only_admitted,
  permanently_closed
};

struct RecoveryServiceLifecycleLabSnapshot {
  RecoveryServiceLifecycleLabState state{
      RecoveryServiceLifecycleLabState::admission_closed};
  bool accepts_recovery_only{false};
  bool can_issue_motion{false};
  bool can_upload_or_load{false};
  bool hardware_command_emitted{false};
  bool physical_stop_confirmed{false};
};

// No owner reference, gateway, draft backend or permission-escalation setter
// escapes. The external service must route ALL recovery requests here, and
// must not open any listener until its real trusted admission checks pass.
class NativeOfflineRecoveryServiceLifecycleLab final {
 public:
  NativeOfflineRecoveryServiceLifecycleLab(
      std::string sqlite_file,
      const DraftQuarantinedReviewLabRequest& independently_verified_review)
      : owner_(std::move(sqlite_file), independently_verified_review) {}

  NativeOfflineRecoveryServiceLifecycleLab(
      const NativeOfflineRecoveryServiceLifecycleLab&) = delete;
  NativeOfflineRecoveryServiceLifecycleLab& operator=(
      const NativeOfflineRecoveryServiceLifecycleLab&) = delete;
  NativeOfflineRecoveryServiceLifecycleLab(
      NativeOfflineRecoveryServiceLifecycleLab&&) = delete;
  NativeOfflineRecoveryServiceLifecycleLab& operator=(
      NativeOfflineRecoveryServiceLifecycleLab&&) = delete;

  // Only opens the OFFLINE recovery routing gate, never normal mission mode.
  // Missing/failed SQLite review may still permit read-only STATUS and a
  // *software* STOP intent, but never recovered PREVIEW, START or upload.
  bool admit_recovery_only() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (state_ != RecoveryServiceLifecycleLabState::admission_closed)
      return false; // No re-admission or reopening after shutdown.
    try {
      const auto s = owner_.status();
      if (!s.quarantined || s.can_start_or_resume || s.can_upload_or_load ||
          owner_.has_active_mission() || owner_.has_live_backend_draft()) {
        close_locked();
        return false;
      }
      state_ = RecoveryServiceLifecycleLabState::recovery_only_admitted;
      return true;
    } catch (...) {
      close_locked();
      return false;
    }
  }

  BackendHttpWireResponse dispatch_recovery_request(
      const BackendTrustedUploadHttpRequest& input,
      const DraftQuarantinedReviewLabRequest& independently_verified_review) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (state_ != RecoveryServiceLifecycleLabState::recovery_only_admitted)
      return blocked();
    try {
      // Sole entry; H5P/H5Q owner dispatch uses its own quarantined gateway.
      // Never trust identities, timestamps or PX4 flags derived from HTTP JSON.
      return owner_.handle_recovery_ingress(input, independently_verified_review);
    } catch (const std::exception&) {
      close_locked();
      return blocked();
    } catch (...) {
      close_locked();
      return blocked();
    }
  }

  // Close admission before retiring the owner. The lock prevents an
  // in-flight test dispatch from outliving a shutdown transition.
  void shutdown() noexcept {
    std::lock_guard<std::mutex> lk(mutex_);
    close_locked();
  }

  RecoveryServiceLifecycleLabSnapshot snapshot() const {
    std::lock_guard<std::mutex> lk(mutex_);
    RecoveryServiceLifecycleLabSnapshot result;
    result.state = state_;
    result.accepts_recovery_only =
        state_ == RecoveryServiceLifecycleLabState::recovery_only_admitted;
    return result;
  }

 private:
  static BackendHttpWireResponse blocked() {
    // 423 is supported by the existing HTTP/1.1 response formatter; this
    // remains an offline decision, NOT an HTTP response transmitted on wire.
    return {423,http_json::error_json("RECOVERY_SERVICE_NOT_ACCEPTING")};
  }
  void close_locked() noexcept {
    state_ = RecoveryServiceLifecycleLabState::permanently_closed;
    try { owner_.retire(); } catch (...) { /* Admission already sealed. */ }
  }

  // Lock ordering: service mutex -> owner mutex, never the inverse.
  // Do not hold this lock around a physical IO or external callback.
  mutable std::mutex mutex_{};
  NativeOfflineRecoveryStartupOwnerLab owner_;
  RecoveryServiceLifecycleLabState state_{
      RecoveryServiceLifecycleLabState::admission_closed};
};

} // namespace dyx::rpp
