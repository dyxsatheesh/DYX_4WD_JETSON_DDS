#pragma once
// DYX Autonomous Phase 2H5S — OFFLINE trusted ingress handoff laboratory.
// This is NOT an authenticator, socket listener, systemd unit or PX4 bridge.
// Only a future privileged adapter may create independent transport evidence.
// Never construct that evidence from Android JSON, HTTP headers or a client.

#include "dyx_rpp_cpp17_core/backend_recovery_service_lifecycle_lab.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>

namespace dyx::rpp {

// Evidence from an independently trusted ingress adapter. These fields are a
// *contract* for integration tests, not cryptographic proof or real attestation.
struct RecoveryTrustedIngressEvidenceLab {
  bool verified_transport_authenticated{false};
  bool verified_can_read_mission{false};
  bool verified_owns_control_lease{false};
  std::string verified_operator_session_id{};
  std::uint64_t trusted_request_id{0}; // Server-assigned, strictly increasing.
  double trusted_monotonic_received_sec{0.0}; // Server clock, not client time.
};

struct RecoveryTrustedIngressSnapshotLab {
  bool admitted{false};
  bool closed_permanently{false};
  std::uint64_t last_accepted_request_id{0};
  bool hardware_command_emitted{false};
};

// Single-process, fail-closed handoff: own a H5R service and PIN the reviewed
// session at construction. No raw, client-supplied principal/time/disarm flag
// ever reaches H5R/H5P. Do not expose H5R directly in a production router.
class NativeOfflineRecoveryTrustedIngressLab final {
 public:
  NativeOfflineRecoveryTrustedIngressLab(
      std::string sqlite_file,
      const DraftQuarantinedReviewLabRequest& independently_verified_review)
      : reviewed_(independently_verified_review),
        reviewed_operator_session_id_(
            independently_verified_review.current_identity.operator_session_id),
        service_(std::move(sqlite_file), independently_verified_review) {}

  NativeOfflineRecoveryTrustedIngressLab(
      const NativeOfflineRecoveryTrustedIngressLab&) = delete;
  NativeOfflineRecoveryTrustedIngressLab& operator=(
      const NativeOfflineRecoveryTrustedIngressLab&) = delete;
  NativeOfflineRecoveryTrustedIngressLab(
      NativeOfflineRecoveryTrustedIngressLab&&) = delete;
  NativeOfflineRecoveryTrustedIngressLab& operator=(
      NativeOfflineRecoveryTrustedIngressLab&&) = delete;

  bool admit_recovery_only() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (closed_ || admitted_) return false;
    if (reviewed_operator_session_id_.empty()) {
      close_locked(); return false;
    }
    try {
      if (!service_.admit_recovery_only()) { close_locked(); return false; }
      admitted_ = true;
      return true;
    } catch (...) { close_locked(); return false; }
  }

  BackendHttpWireResponse dispatch(
      const BackendTrustedUploadHttpRequest& raw_client_request,
      const RecoveryTrustedIngressEvidenceLab& independent_evidence) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!admitted_ || closed_) return deny(423, "RECOVERY_INGRESS_NOT_ACCEPTING");

    // Even a STOP must be authenticated; an authenticated STOP remains a
    // software intent only, never a confirmed physical braking event.
    if (!independent_evidence.verified_transport_authenticated)
      return deny(401, "TRUSTED_TRANSPORT_AUTH_REQUIRED");
    if (independent_evidence.verified_operator_session_id.empty() ||
        independent_evidence.verified_operator_session_id !=
            reviewed_operator_session_id_)
      return deny(403, "TRUSTED_SESSION_MISMATCH");
    if (independent_evidence.trusted_request_id == 0 ||
        independent_evidence.trusted_request_id <= last_request_id_)
      return deny(409, "RECOVERY_REQUEST_REPLAY");
    if (!std::isfinite(independent_evidence.trusted_monotonic_received_sec) ||
        independent_evidence.trusted_monotonic_received_sec < 0.0 ||
        independent_evidence.trusted_monotonic_received_sec < last_received_sec_)
      return deny(400, "INVALID_TRUSTED_CLOCK");
    const auto& req = raw_client_request.http;
    if (req.path.size() > 512 || req.body.size() > 1024)
      return deny(413, "RECOVERY_REQUEST_TOO_LARGE");
    if (req.method.size() > 8 || req.path.empty())
      return deny(400, "INVALID_RECOVERY_REQUEST");

    // AFTER checking evidence, overwrite all caller-controlled trusted facts.
    // PX4 disarm assertions from the input are ALWAYS discarded.
    BackendTrustedUploadHttpRequest verified = raw_client_request;
    verified.http.principal = {};
    verified.http.principal.authenticated = true;
    verified.http.principal.operator_session_id =
        independent_evidence.verified_operator_session_id;
    verified.http.principal.can_read_mission =
        independent_evidence.verified_can_read_mission;
    verified.http.principal.owns_control_lease =
        independent_evidence.verified_owns_control_lease;
    verified.http.host_received_sec =
        independent_evidence.trusted_monotonic_received_sec;
    verified.trusted_px4_disarmed = false;

    // Advance the replay fence BEFORE dispatch, including denied commands.
    // No exception or HTTP error can undo an accepted trusted request ID.
    last_request_id_ = independent_evidence.trusted_request_id;
    last_received_sec_ = independent_evidence.trusted_monotonic_received_sec;
    try {
      auto reply = service_.dispatch_recovery_request(verified, reviewed_);
      if (reply.physical_command_emitted || reply.network_io_performed ||
          reply.body.size() > 16384) {
        close_locked(); return deny(423, "RECOVERY_OUTPUT_INVARIANT_FAILED");
      }
      return reply;
    } catch (...) {
      close_locked();
      return deny(423, "RECOVERY_INGRESS_FAILED_CLOSED");
    }
  }

  void shutdown() noexcept {
    std::lock_guard<std::mutex> lk(mutex_);
    close_locked();
  }
  RecoveryTrustedIngressSnapshotLab snapshot() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return {admitted_ && !closed_, closed_, last_request_id_, false};
  }

 private:
  static BackendHttpWireResponse deny(int status, const char* code) {
    return {status, http_json::error_json(code)};
  }
  void close_locked() noexcept {
    closed_ = true;
    admitted_ = false;
    try { service_.shutdown(); } catch (...) { /* irrevocably sealed */ }
  }
  // Lock ordering: trusted-ingress mutex -> H5R -> H5Q. Never reverse.
  mutable std::mutex mutex_{};
  const DraftQuarantinedReviewLabRequest reviewed_;
  const std::string reviewed_operator_session_id_;
  NativeOfflineRecoveryServiceLifecycleLab service_;
  bool admitted_{false};
  bool closed_{false};
  std::uint64_t last_request_id_{0};
  double last_received_sec_{0.0};
};

} // namespace dyx::rpp
