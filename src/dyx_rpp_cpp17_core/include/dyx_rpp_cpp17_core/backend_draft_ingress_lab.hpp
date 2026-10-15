#pragma once
// DYX Autonomous Phase 2H4T: OFFLINE in-process exercise of the privileged
// Android-draft -> HTTP parser -> mission backend boundary. This is a TEST LAB,
// NOT a network ingress, authenticator, TLS implementation, trusted PX4 reader,
// ROS 2 / DDS publisher, or physical actuator driver. The privileged facts
// below must come from independent systems in any future production adapter.

#include "dyx_rpp_cpp17_core/backend_android_draft_coordinator.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace dyx::rpp {

struct DraftIngressLabConfig {
  double max_auth_age_sec{2.0};
  double max_disarm_age_sec{0.5};
  void validate() const {
    if (!std::isfinite(max_auth_age_sec) || max_auth_age_sec < 0.1 ||
        max_auth_age_sec > 30.0 || !std::isfinite(max_disarm_age_sec) ||
        max_disarm_age_sec < 0.05 || max_disarm_age_sec > 5.0)
      throw std::invalid_argument("invalid offline ingress lab limits");
  }
};

// UNFORGEABILITY IS NOT IMPLEMENTED HERE. The lab can only enforce the
// consistency of evidence passed by the caller. Do not populate this struct
// directly from HTTP headers, JSON or Android UI state in a real application.
struct DraftIngressLabEvidence {
  BackendVerifiedPrincipal independently_verified_principal{};
  bool independently_verified_transport{false};
  std::uint64_t independently_verified_session_epoch{0};
  std::uint64_t independently_verified_ingress_nonce{0};
  double independently_verified_auth_sec{std::numeric_limits<double>::quiet_NaN()};
  double trusted_received_sec{std::numeric_limits<double>::quiet_NaN()};
  bool independently_verified_px4_source{false};
  bool px4_disarmed{false};
  double independently_observed_px4_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct DraftIngressLabOutcome {
  bool trusted_reply_provenance{false};
  bool reached_backend{false};
  bool network_io_performed{false};
  bool hardware_command_emitted{false};
  std::string code{"INGRESS_DENIED"};
  BackendHttpWireResponse wire{403,"{\"ok\":false,\"code\":\"INGRESS_DENIED\"}"};
};

class NativeOfflineDraftIngressLab final {
 public:
  NativeOfflineDraftIngressLab(NativeBackendUploadHttpWire& backend,
                              DraftIngressLabConfig cfg={})
      : backend_(backend), cfg_(cfg) { cfg_.validate(); }

  DraftIngressLabOutcome mutation(const AndroidDraftEncodedRequest& request,
                                  const DraftIngressLabEvidence& evidence) {
    // Reject forged route/shape BEFORE calling the backend, never trust a
    // mutation encoded by the Android UI to specify its own authority.
    if (!request.valid || request.method!="POST" ||
        !android_draft_detail::draft_mutation_route(request.path) ||
        request.sequence==0 || request.body.empty() ||
        request.network_io_performed || request.hardware_command_emitted)
      return deny("INGRESS_INVALID_MUTATION");
    if (!authorize(evidence,true)) return deny("INGRESS_AUTHORITY_DENIED");
    // The request identity is deliberately NOT derived from an untrusted
    // principal/session field in the JSON body.
    BackendTrustedUploadHttpRequest input;
    input.http.method=request.method;
    input.http.path=request.path;
    input.http.body=request.body;
    input.http.principal=evidence.independently_verified_principal;
    input.http.host_received_sec=evidence.trusted_received_sec;
    input.trusted_px4_disarmed=true;
    return deliver(input);
  }

  DraftIngressLabOutcome snapshot(const AndroidDraftSyncRequest& request,
                                  const DraftIngressLabEvidence& evidence) {
    if (!request.valid || request.method!="GET" || !request.body.empty() ||
        request.request_id==0 || request.network_io_performed ||
        request.hardware_command_emitted ||
        !valid_native_mission_run_id(request.mission_run_id) ||
        request.path != std::string(kBackendMissionDraftPath) +
                        "?mission_run_id=" + request.mission_run_id)
      return deny("INGRESS_INVALID_SNAPSHOT");
    if (!authorize(evidence,false)) return deny("INGRESS_AUTHORITY_DENIED");
    BackendTrustedUploadHttpRequest input;
    input.http.method="GET";
    input.http.path=request.path;
    input.http.body.clear();
    input.http.principal=evidence.independently_verified_principal;
    input.http.host_received_sec=evidence.trusted_received_sec;
    // A GET does not require or imply PX4 disarm attestation.
    input.trusted_px4_disarmed=false;
    return deliver(input);
  }

  bool fault_latched() const { return clock_fault_; }
  std::uint64_t last_ingress_nonce() const { return last_nonce_; }
  bool session_bound() const { return bound_; }

 private:
  bool authorize(const DraftIngressLabEvidence& e, bool mutation) {
    if (clock_fault_) return false;
    const auto now=e.trusted_received_sec;
    if (!std::isfinite(now) || now<0.0 || now<last_clock_) {
      clock_fault_=true;
      return false;
    }
    last_clock_=now; // host monotonic regression permanently fails closed
    const auto& p=e.independently_verified_principal;
    if (!e.independently_verified_transport || !p.authenticated ||
        !valid_native_mission_run_id(p.operator_session_id) ||
        e.independently_verified_session_epoch==0 ||
        e.independently_verified_ingress_nonce==0 ||
        !std::isfinite(e.independently_verified_auth_sec) ||
        e.independently_verified_auth_sec<0.0 ||
        e.independently_verified_auth_sec>now ||
        now-e.independently_verified_auth_sec>cfg_.max_auth_age_sec ||
        (mutation ? !p.owns_control_lease : !p.can_read_mission))
      return false;
    if (mutation && (!e.independently_verified_px4_source || !e.px4_disarmed ||
        !std::isfinite(e.independently_observed_px4_sec) ||
        e.independently_observed_px4_sec<0.0 ||
        e.independently_observed_px4_sec>now ||
        now-e.independently_observed_px4_sec>cfg_.max_disarm_age_sec))
      return false;
    if (bound_ && (p.operator_session_id!=bound_session_ ||
                   e.independently_verified_session_epoch!=bound_epoch_))
      return false;
    if (e.independently_verified_ingress_nonce<=last_nonce_) return false;
    if (!bound_) {
      bound_=true;
      bound_session_=p.operator_session_id;
      bound_epoch_=e.independently_verified_session_epoch;
    }
    // Monotonic nonce is consumed even for a backend 4xx. Replaying a rejected
    // mutation must not enable it later if backend conditions change.
    last_nonce_=e.independently_verified_ingress_nonce;
    return true;
  }
  DraftIngressLabOutcome deliver(const BackendTrustedUploadHttpRequest& input) {
    DraftIngressLabOutcome result;
    result.reached_backend=true;
    result.trusted_reply_provenance=true; // lab-only simulated verified channel
    result.wire=backend_.handle(input);
    result.code="INGRESS_VERIFIED_BACKEND_DECISION";
    return result;
  }
  static DraftIngressLabOutcome deny(const char* reason) {
    DraftIngressLabOutcome result;
    result.code=reason;
    result.wire={403,http_json::error_json(reason)};
    return result;
  }
  NativeBackendUploadHttpWire& backend_;
  DraftIngressLabConfig cfg_{};
  bool bound_{false};
  std::string bound_session_{};
  std::uint64_t bound_epoch_{0};
  std::uint64_t last_nonce_{0};
  double last_clock_{0.0};
  bool clock_fault_{false};
};

// A lab fixture may shape its return as verified transport metadata ONLY when
// independently_authenticated evidence passed through the ingress gate.
inline AndroidTrustedDraftHttpReply draft_ingress_lab_mutation_reply(
    const AndroidDraftEncodedRequest& request,
    const DraftIngressLabOutcome& result,
    double sent_sec,double received_sec) {
  AndroidTrustedDraftHttpReply reply;
  reply.wire=result.wire;
  reply.verified_authenticated_transport=result.trusted_reply_provenance;
  reply.verified_request_sequence=request.sequence;
  reply.verified_request_path=request.path;
  reply.verified_sent_sec=sent_sec;
  reply.verified_received_sec=received_sec;
  return reply;
}
inline AndroidTrustedDraftSyncReply draft_ingress_lab_sync_reply(
    const AndroidDraftSyncRequest& request,
    const DraftIngressLabOutcome& result,
    double sent_sec,double received_sec) {
  AndroidTrustedDraftSyncReply reply;
  reply.wire=result.wire;
  reply.verified_authenticated_transport=result.trusted_reply_provenance;
  reply.verified_request_id=request.request_id;
  reply.verified_request_path=request.path;
  reply.verified_sent_sec=sent_sec;
  reply.verified_received_sec=received_sec;
  return reply;
}

} // namespace dyx::rpp
