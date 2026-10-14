#pragma once
// Phase 2H4I: OFFLINE ONLY control-lease and trusted-ingress contract.
// This is NOT authentication: assertions MUST come from a separately verified
// TLS/identity component, NEVER request headers, JSON, Android, or loopback.
// No sockets, DDS publication, PX4 commands, motor, spray, or laser outputs.

#include "dyx_rpp_cpp17_core/backend_http_wire.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace dyx::rpp {

struct TrustedIngressIdentity {
  // These assertions have NO value if populated by an untrusted caller.
  bool authentication_verified{false};
  bool read_scope_verified{false};
  bool control_scope_verified{false};
  std::string operator_id{};
  std::string session_id{};
};

struct ControlSessionConfig {
  double lease_duration_sec{10.0};
  void validate() const {
    if (!std::isfinite(lease_duration_sec) ||
        lease_duration_sec < 0.5 || lease_duration_sec > 60.0)
      throw std::invalid_argument("invalid control lease duration");
  }
};

enum class IngressDecisionCode {
  permitted, authentication_required, invalid_identity, read_scope_required,
  control_scope_required, lease_busy, lease_required, lease_expired,
  generation_mismatch, sequence_replay, invalid_sequence, clock_fault
};
inline const char* ingress_code_name(IngressDecisionCode c) {
  switch (c) {
    case IngressDecisionCode::permitted: return "INGRESS_PERMITTED";
    case IngressDecisionCode::authentication_required: return "AUTHENTICATION_REQUIRED";
    case IngressDecisionCode::invalid_identity: return "INVALID_VERIFIED_IDENTITY";
    case IngressDecisionCode::read_scope_required: return "READ_SCOPE_REQUIRED";
    case IngressDecisionCode::control_scope_required: return "CONTROL_SCOPE_REQUIRED";
    case IngressDecisionCode::lease_busy: return "CONTROL_LEASE_BUSY";
    case IngressDecisionCode::lease_required: return "CONTROL_LEASE_REQUIRED";
    case IngressDecisionCode::lease_expired: return "CONTROL_LEASE_EXPIRED";
    case IngressDecisionCode::generation_mismatch: return "LEASE_GENERATION_MISMATCH";
    case IngressDecisionCode::sequence_replay: return "COMMAND_SEQUENCE_REPLAY";
    case IngressDecisionCode::invalid_sequence: return "INVALID_SEQUENCE";
    case IngressDecisionCode::clock_fault: return "HOST_CLOCK_FAULT";
  }
  return "INGRESS_INTERNAL_ERROR";
}

struct IngressDecision {
  IngressDecisionCode code{IngressDecisionCode::authentication_required};
  BackendVerifiedPrincipal principal{};
  std::uint64_t generation{0};
  bool allowed() const { return code == IngressDecisionCode::permitted; }
  bool hardware_command_emitted{false}; // invariant
};

enum class IngressOperation { read, control, stop };

class NativeOfflineControlSessions final {
 public:
  explicit NativeOfflineControlSessions(ControlSessionConfig cfg = {}) : cfg_(cfg) {
    cfg_.validate();
  }

  // Every call uses a HOST monotonic clock. On regression, latch a fault and
  // revoke the lease; no automatic recovery from potentially reused evidence.
  IngressDecision acquire(const TrustedIngressIdentity& id, double now_sec) {
    if (!identity_ok(id)) return deny(identity_failure(id));
    if (!id.control_scope_verified) return deny(IngressDecisionCode::control_scope_required);
    if (!clock_ok(now_sec)) return deny(IngressDecisionCode::clock_fault);
    expire_if_needed(now_sec);
    if (active_) return deny(IngressDecisionCode::lease_busy);
    if (!advance_generation()) return deny(IngressDecisionCode::clock_fault);
    active_=true;
    operator_id_=id.operator_id;
    session_id_=id.session_id;
    deadline_=now_sec+cfg_.lease_duration_sec;
    highest_sequence_=0;
    return granted(id, true);
  }

  IngressDecision renew(const TrustedIngressIdentity& id,
                        std::uint64_t generation, double now_sec) {
    if (!identity_ok(id)) return deny(identity_failure(id));
    if (!id.control_scope_verified) return deny(IngressDecisionCode::control_scope_required);
    if (!clock_ok(now_sec)) return deny(IngressDecisionCode::clock_fault);
    if (expire_if_needed(now_sec)) return deny(IngressDecisionCode::lease_expired);
    if (!active_) return deny(IngressDecisionCode::lease_required);
    if (generation != generation_ || !is_owner(id))
      return deny(IngressDecisionCode::generation_mismatch);
    deadline_=now_sec+cfg_.lease_duration_sec;
    return granted(id, true);
  }

  IngressDecision authorize(const TrustedIngressIdentity& id,
                            IngressOperation op, std::uint64_t bound_generation,
                            std::uint64_t sequence, double now_sec) {
    if (!identity_ok(id)) return deny(identity_failure(id));
    // STOP remains available to a VERIFIED control-scope operator without an
    // active lease and regardless of the clock, as an advisory STOP request.
    // The mission gateway independently verifies active run/session identity.
    if (op == IngressOperation::stop) {
      if (!id.control_scope_verified) return deny(IngressDecisionCode::control_scope_required);
      if (!clock_ok(now_sec)) {/* fault latch, but never block authenticated STOP */}
      if (sequence==0) return deny(IngressDecisionCode::invalid_sequence);
      return granted(id, false);
    }
    if (!clock_ok(now_sec)) return deny(IngressDecisionCode::clock_fault);
    if (op == IngressOperation::read) {
      if (!id.read_scope_verified) return deny(IngressDecisionCode::read_scope_required);
      return granted(id, false);
    }
    if (!id.control_scope_verified) return deny(IngressDecisionCode::control_scope_required);
    if (expire_if_needed(now_sec)) return deny(IngressDecisionCode::lease_expired);
    if (!active_) return deny(IngressDecisionCode::lease_required);
    if (bound_generation==0 || bound_generation!=generation_ || !is_owner(id))
      return deny(IngressDecisionCode::generation_mismatch);
    if (sequence==0) return deny(IngressDecisionCode::invalid_sequence);
    if (sequence<=highest_sequence_) return deny(IngressDecisionCode::sequence_replay);
    // Reserve the sequence BEFORE the application safety decision; a denied
    // command cannot become accepted via retransmission after RTK recovery.
    highest_sequence_=sequence;
    return granted(id, true);
  }

  void revoke() {
    active_=false; operator_id_.clear();session_id_.clear();
    deadline_=0;highest_sequence_=0;
    (void)advance_generation();
  }
  // Operator must be verified AND the vehicle must be independently confirmed
  // DISARMED before a regression fault can be cleared. No implicit re-arm.
  bool reset_clock_fault_disarmed(bool authorized, bool px4_disarmed,
                                  double new_host_now_sec) {
    if (!authorized || !px4_disarmed || !std::isfinite(new_host_now_sec) ||
        new_host_now_sec<0.0) return false;
    revoke();
    if (generation_fault_) return false;
    clock_fault_=false;
    last_now_=new_host_now_sec;
    return true;
  }
  bool has_active_lease() const { return active_; }
  std::uint64_t generation() const { return generation_; }
  bool fault_latched() const { return clock_fault_ || generation_fault_; }

 private:
  static bool identity_ok(const TrustedIngressIdentity& id) {
    return id.authentication_verified &&
           valid_native_mission_run_id(id.operator_id) &&
           valid_native_mission_run_id(id.session_id);
  }
  static IngressDecisionCode identity_failure(const TrustedIngressIdentity& id) {
    return id.authentication_verified ? IngressDecisionCode::invalid_identity :
        IngressDecisionCode::authentication_required;
  }
  bool clock_ok(double now) {
    if (clock_fault_ || generation_fault_) return false;
    if (!std::isfinite(now) || now<0.0 || now<last_now_) {
      clock_fault_=true;
      revoke();
      return false;
    }
    last_now_=now;
    return true;
  }
  bool advance_generation() {
    if (generation_==std::numeric_limits<std::uint64_t>::max()) {
      generation_fault_=true;
      return false;
    }
    ++generation_;return true;
  }
  bool expire_if_needed(double now) {
    if (!active_ || now<deadline_) return false; // expiry is exclusive
    revoke();return true;
  }
  bool is_owner(const TrustedIngressIdentity& id) const {
    return id.session_id==session_id_ && id.operator_id==operator_id_;
  }
  IngressDecision deny(IngressDecisionCode c) const { return {c, {}, generation_}; }
  IngressDecision granted(const TrustedIngressIdentity& id, bool lease) const {
    return {IngressDecisionCode::permitted,
        {true,id.read_scope_verified,lease,id.session_id}, generation_};
  }
  ControlSessionConfig cfg_{};
  double last_now_{0.0}, deadline_{0.0};
  bool active_{false}, clock_fault_{false}, generation_fault_{false};
  std::string operator_id_{},session_id_{};
  std::uint64_t generation_{0}, highest_sequence_{0};
};

// Not connected to NativeLoopbackReadOnlyHttpServer. Only test callers can
// inject independently authenticated identities. A production network ingress
// must implement TLS, real credential checks, origin policy and concurrency.
class NativeOfflineTrustedRequestAdapter final {
 public:
  NativeOfflineTrustedRequestAdapter(NativeOfflineControlSessions& sessions,
                                     NativeBackendHttpWire& wire)
      : sessions_(sessions), wire_(wire) {}

  BackendHttpWireResponse handle(const TrustedIngressIdentity& verified,
                                 std::uint64_t trusted_bound_generation,
                                 const BackendTrustedHttpRequest& untrusted_request) {
    // Never trust authorization booleans or session IDs inside the request.
    BackendTrustedHttpRequest req=untrusted_request;
    req.principal={};
    const bool read=req.method=="GET" &&
       (req.path==kBackendMissionStatusPath || req.path==kBackendLiveStatusPath);
    const bool post=req.method=="POST" &&
       (req.path==kBackendMissionStartPath || req.path==kBackendMissionPausePath ||
        req.path==kBackendMissionResumePath || req.path==kBackendMissionNextPath ||
        req.path==kBackendMissionSkipPath || req.path==kBackendMissionStopPath);
    if (!read && !post) return failure(405,"OFFLINE_ROUTE_NOT_ALLOWED");
    std::uint64_t sequence=0;
    if (post) {
      const auto parsed=http_detail::StrictCommandParser(req.body).parse();
      if (!parsed.valid) return failure(400,"INVALID_COMMAND_JSON");
      sequence=parsed.sequence;
    }
    const auto op=read ? IngressOperation::read :
        (req.path==kBackendMissionStopPath ? IngressOperation::stop : IngressOperation::control);
    const auto decision=sessions_.authorize(verified,op,trusted_bound_generation,
                                             sequence,req.host_received_sec);
    if (!decision.allowed()) {
      const int status= decision.code==IngressDecisionCode::authentication_required ? 401 :
          decision.code==IngressDecisionCode::clock_fault ? 412 : 403;
      return failure(status,ingress_code_name(decision.code));
    }
    req.principal=decision.principal;
    return wire_.handle(req); // existing gate remains decision-only
  }
 private:
  static BackendHttpWireResponse failure(int status, const char* code) {
    return {status,http_json::error_json(code)};
  }
  NativeOfflineControlSessions& sessions_;
  NativeBackendHttpWire& wire_;
};

} // namespace dyx::rpp
