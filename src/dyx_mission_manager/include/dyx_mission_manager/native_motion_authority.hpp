#pragma once
// DYX 117: fail-closed, transport-independent positive-command decision model.
// A read-only software verification facility: never emits PX4/spray commands.
// A valid software lease is not a hardware permission-to-drive capability.
#include "dyx_mission_manager/native_mission_transition_gate.hpp"
#include "dyx_mission_manager/native_positive_command_preflight.hpp"
#include <cmath>
#include <cstdint>
#include <string>

namespace dyx::http_backend {

struct MotionLeaseSnapshot {
  std::string session_id{};
  std::string run_id{};
  std::uint64_t epoch{0};
  double issued_monotonic_sec{-1.0};
  double expires_monotonic_sec{-1.0};
  bool server_verified{false};
};

struct AuthoritySafetySnapshot {
  PositiveCommandEvidence facts{};
  double collected_monotonic_sec{-1.0};
  std::uint64_t authority_epoch{0};
  bool trusted_adapter_bundle{false};
};

struct AuthorityDecision {
  bool stage_valid{false};
  bool lease_valid{false};
  bool evidence_current{false};
  bool diagnostics_ready{false};
  bool software_eligible{false};
  bool motion_authorized{false}; // Always false: physical commissioning absent.
  const char* reason{"NOT_EVALUATED"};
};

class NativeMotionAuthority final {
 public:
  static constexpr double kMaxEvidenceAgeSec = 0.25;
  static constexpr double kMaxLeaseSeconds = 30.0;

  // Input is from a trusted server-side adapter, NEVER an HTTP JSON body.
  // It can be used for offline/bench classification, not physical actuation.
  void observe_lease(const MotionLeaseSnapshot& lease) {
    if (stop_latched_) return;
    if (!lease.server_verified || lease.session_id.empty() || lease.run_id.empty() ||
        lease.epoch == 0 || !std::isfinite(lease.issued_monotonic_sec) ||
        !std::isfinite(lease.expires_monotonic_sec) ||
        lease.expires_monotonic_sec <= lease.issued_monotonic_sec ||
        lease.expires_monotonic_sec-lease.issued_monotonic_sec > kMaxLeaseSeconds) {
      revoke();
      return;
    }
    // Replacement epochs must increase: replaying or downgrading an epoch fails.
    if (lease.epoch <= highest_epoch_seen_) {
      revoke();
      return;
    }
    highest_epoch_seen_ = lease.epoch;
    lease_ = lease;
  }

  void observe_safety(const AuthoritySafetySnapshot& snapshot) {
    if (stop_latched_) return;
    evidence_ = snapshot;
  }

  void revoke() {
    lease_ = {};
    evidence_ = {};
  }

  void stop() {
    stop_latched_ = true;
    revoke();
  }

  bool stop_latched() const noexcept { return stop_latched_; }

  AuthorityDecision assess(const dyx::rpp::BackendMissionApiRequest& request,
                           dyx::rpp::MissionStage stage,
                           double now_sec) const {
    AuthorityDecision d{};
    if (stop_latched_) {d.reason="STOP_LATCHED";return d;}
    if (!std::isfinite(now_sec) || now_sec < 0.0) {
      d.reason="INVALID_MONOTONIC_TIME";return d;
    }
    if (!request.principal.authenticated ||
        request.principal.operator_session_id.empty() ||
        request.mission_run_id.empty()) {
      d.reason="UNVERIFIED_PRINCIPAL";return d;
    }
    auto stage_check = request;
    // Current transition classifier checks the lease flag as well as stage;
    // this temporary true value is local to classification, never dispatched.
    stage_check.principal.owns_control_lease = true;
    const auto transition=assess_mission_transition(stage_check, stage, false);
    d.stage_valid = transition.eligible_stage;
    if (!d.stage_valid) {d.reason="INVALID_STAGE_OR_ACTION";return d;}
    d.lease_valid = lease_.server_verified && lease_.epoch!=0 &&
        lease_.session_id==request.principal.operator_session_id &&
        lease_.run_id==request.mission_run_id &&
        now_sec>=lease_.issued_monotonic_sec &&
        now_sec<lease_.expires_monotonic_sec &&
        lease_.expires_monotonic_sec-lease_.issued_monotonic_sec<=kMaxLeaseSeconds;
    if (!d.lease_valid) {d.reason="CONTROL_LEASE_INVALID";return d;}
    d.evidence_current = evidence_.trusted_adapter_bundle &&
        evidence_.authority_epoch==lease_.epoch &&
        std::isfinite(evidence_.collected_monotonic_sec) &&
        now_sec>=evidence_.collected_monotonic_sec &&
        now_sec-evidence_.collected_monotonic_sec<=kMaxEvidenceAgeSec;
    if (!d.evidence_current) {d.reason="SAFETY_EVIDENCE_STALE_OR_UNTRUSTED";return d;}
    const auto preflight=assess_positive_command_preflight(evidence_.facts);
    d.diagnostics_ready=preflight.diagnostic_preconditions_met;
    if (!d.diagnostics_ready) {d.reason=preflight.first_blocker;return d;}
    d.software_eligible=true;
    d.motion_authorized=false; // No physical activation in this patch.
    d.reason="SOFTWARE_ELIGIBLE_HARDWARE_LOCKED";
    return d;
  }
 private:
  MotionLeaseSnapshot lease_{};
  std::uint64_t highest_epoch_seen_{0};
  AuthoritySafetySnapshot evidence_{};
  bool stop_latched_{false};
};

} // namespace dyx::http_backend
