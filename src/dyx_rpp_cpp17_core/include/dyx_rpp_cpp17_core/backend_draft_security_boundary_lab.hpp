#pragma once
// DYX Autonomous Phase 2H4V — OFFLINE, non-cryptographic trust-source boundary.
// Absolutely no sockets, TLS, real operator authentication, trusted PX4/DDS
// connection, persistence, or physical actuation are implemented here.
// A real adapter MUST implement independently authenticated session / lease
// verification and an independent, live PX4 safety-state reader.

#include "dyx_rpp_cpp17_core/backend_draft_http_framing_lab.hpp"
#include <cstdint>
#include <string>

namespace dyx::rpp {

// These are privileged-source OUTPUT types, never HTTP/JSON input types.
// A caller implementing this interface is still trusted in the lab, not
// cryptographically proven. Do not expose the source to a network handler.
struct DraftBoundaryLabSession {
  bool verified_transport{false};
  BackendVerifiedPrincipal principal{};
  std::uint64_t session_epoch{0};
  std::uint64_t ingress_nonce{0};
  double verified_auth_sec{0.0};
};

struct DraftBoundaryLabPx4 {
  bool verified_px4_source{false};
  bool disarmed{false};
  double observed_sec{0.0};
};

class DraftBoundaryLabTrustedSource {
 public:
  virtual ~DraftBoundaryLabTrustedSource() = default;
  virtual bool monotonic_receive_time(double& seconds) = 0;
  virtual bool verified_session(DraftBoundaryLabSession& session) = 0;
  virtual bool independently_observed_px4(DraftBoundaryLabPx4& px4) = 0;
};

class NativeOfflineDraftSecurityBoundaryLab final {
 public:
  NativeOfflineDraftSecurityBoundaryLab(NativeOfflineDraftHttpFramingLab& framing,
                                       DraftBoundaryLabTrustedSource& source,
                                       DraftHttpFramingLabConfig cfg={})
      : framing_(framing), source_(source), cfg_(cfg) { cfg_.validate(); }

  DraftIngressLabOutcome mutation(const std::string& raw_http,
                                  const AndroidDraftEncodedRequest& prepared) {
    if (!preflight(raw_http, prepared.method, prepared.path, prepared.body) ||
        !prepared.valid || prepared.sequence == 0 ||
        prepared.network_io_performed || prepared.hardware_command_emitted ||
        prepared.method != "POST" ||
        !android_draft_detail::draft_mutation_route(prepared.path))
      return draft_http_framing_detail::deny();
    DraftIngressLabEvidence evidence;
    if (!session_evidence(evidence)) return unavailable();
    DraftBoundaryLabPx4 px4;
    if (!source_.independently_observed_px4(px4)) return unavailable();
    evidence.independently_verified_px4_source=px4.verified_px4_source;
    evidence.px4_disarmed=px4.disarmed;
    evidence.independently_observed_px4_sec=px4.observed_sec;
    // The existing H4U frame/JSON binding and H4T authority/nonce gate are
    // the ONLY way this proof reaches the backend.
    return framing_.mutation(raw_http,prepared,evidence);
  }

  DraftIngressLabOutcome snapshot(const std::string& raw_http,
                                  const AndroidDraftSyncRequest& prepared) {
    if (!preflight(raw_http, prepared.method, prepared.path, prepared.body) ||
        !prepared.valid || prepared.request_id == 0 ||
        prepared.network_io_performed || prepared.hardware_command_emitted ||
        prepared.method != "GET" || !prepared.body.empty())
      return draft_http_framing_detail::deny();
    DraftIngressLabEvidence evidence;
    if (!session_evidence(evidence)) return unavailable();
    // Read-only GET must NOT consult PX4 or fabricate a disarm attestation.
    return framing_.snapshot(raw_http,prepared,evidence);
  }
 private:
  bool preflight(const std::string& raw, const std::string& method,
                 const std::string& path,const std::string& body) const {
    const auto parsed=draft_http_framing_detail::parse(raw,cfg_);
    return parsed.valid && parsed.method==method &&
           parsed.path==path && parsed.body==body;
  }
  bool session_evidence(DraftIngressLabEvidence& out) {
    double now=0.0;
    DraftBoundaryLabSession s;
    if (!source_.monotonic_receive_time(now) ||
        !source_.verified_session(s)) return false;
    out.independently_verified_principal=s.principal;
    out.independently_verified_transport=s.verified_transport;
    out.independently_verified_session_epoch=s.session_epoch;
    out.independently_verified_ingress_nonce=s.ingress_nonce;
    out.independently_verified_auth_sec=s.verified_auth_sec;
    out.trusted_received_sec=now;
    return true;
  }
  static DraftIngressLabOutcome unavailable() {
    DraftIngressLabOutcome result;
    result.code="BOUNDARY_LAB_SOURCE_UNAVAILABLE";
    result.wire={503,http_json::error_json(result.code)};
    return result;
  }
  NativeOfflineDraftHttpFramingLab& framing_;
  DraftBoundaryLabTrustedSource& source_;
  DraftHttpFramingLabConfig cfg_{};
};

} // namespace dyx::rpp
