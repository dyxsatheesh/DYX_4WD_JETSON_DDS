#pragma once
// DYX Autonomous Phase 2H5N: isolated, NON-PRODUCTION recovery startup owner.
// Solely owns one backend + gateway pair. The constructor latches quarantine
// BEFORE the object can be used and performs ONE offline H5K admission check.
// No HTTP listener, OS service, physical STOP, PX4 or DDS connection exists.
// All identity/review claims must be independently verified by production ingress.

#include "dyx_rpp_cpp17_core/backend_draft_quarantined_boot_guard_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_http_wire.hpp"
#include "dyx_rpp_cpp17_core/backend_upload_http_wire.hpp"
#include <cmath>
#include <exception>
#include <string>
#include <utility>

namespace dyx::rpp {

struct RecoveryStartupOwnerLabStatus {
  DraftQuarantinedBootGuardLabState review_state{
      DraftQuarantinedBootGuardLabState::locked_permanently};
  DraftQuarantinedReviewLabState audit_state{
      DraftQuarantinedReviewLabState::denied};
  bool quarantined{true};
  bool read_only_preview_possible{false};
  bool can_upload_or_load{false};
  bool can_start_or_resume{false};
  bool physical_stop_confirmed{false};
  bool hardware_command_emitted{false};
};

class NativeOfflineRecoveryStartupOwnerLab final {
 public:
  NativeOfflineRecoveryStartupOwnerLab(
      std::string sqlite_file,
      const DraftQuarantinedReviewLabRequest& independently_verified_review)
      : gateway_(), backend_(gateway_),
        boot_(std::move(sqlite_file), gateway_, backend_) {
    // Latch BEFORE even attempting to read or trust any persisted content.
    // No references to these controlled objects ever escape this owner.
    gateway_.latch_recovery_quarantine();
    boot_admission_ = boot_.begin(independently_verified_review);
  }

  NativeOfflineRecoveryStartupOwnerLab(const NativeOfflineRecoveryStartupOwnerLab&)
      = delete;
  NativeOfflineRecoveryStartupOwnerLab& operator=(
      const NativeOfflineRecoveryStartupOwnerLab&) = delete;
  NativeOfflineRecoveryStartupOwnerLab(NativeOfflineRecoveryStartupOwnerLab&&)
      = delete;
  NativeOfflineRecoveryStartupOwnerLab& operator=(
      NativeOfflineRecoveryStartupOwnerLab&&) = delete;

  RecoveryStartupOwnerLabStatus status() const {
    RecoveryStartupOwnerLabStatus out;
    out.review_state = boot_.state();
    out.audit_state = boot_admission_.audit;
    out.quarantined = gateway_.recovery_quarantined();
    out.read_only_preview_possible =
        boot_.state() == DraftQuarantinedBootGuardLabState::read_only_preview;
    return out;
  }

  // Never forwards upload/update/load; H5K permits a checked, authorized,
  // body-free GET /api/v1/mission/draft only. Its response is metadata only.
  BackendMissionUploadResponse handle_draft(
      const BackendMissionUploadRequest& req,
      const DraftQuarantinedReviewLabRequest& verified_review) {
    return boot_.handle(req, verified_review);
  }

  // Independently authenticated STOP remains available as a SOFTWARE intent.
  // All other commands go through the one-way H5L gateway quarantine.
  // This method does not send a real motor, PX4 or DDS command.
  ClientCommandResult handle_command(const ClientCommandRequest& req) {
    return gateway_.dispatch(req);
  }

  // H5O: offline typed HTTP dispatch on the EXACT owner-owned gateway.
  // This is an integration boundary for a future privileged listener, NOT a
  // listener, TLS/auth verifier or physical STOP. Raw network input must never
  // populate BackendVerifiedPrincipal or host_received_sec directly.
  BackendHttpWireResponse handle_http(const BackendTrustedHttpRequest& request) {
    // Never publish a different/unquarantined gateway through any HTTP route.
    NativeBackendMissionApi mission_api(gateway_);
    NativeBackendLiveStatus live_status(gateway_);
    NativeBackendHttpWire wire(mission_api, live_status);
    return wire.handle(request);
  }

  // H5K's *fresh SQLite-verified*, read-only recovered PREVIEW needs a
  // distinct route. Ordinary NativeBackendUploadHttpWire is deliberately NOT
  // used: it would let a caller attempt a direct mission-upload backend.
  // The caller must supply independently verified identity/session facts.
  BackendHttpWireResponse handle_recovered_draft_http(
      const BackendTrustedUploadHttpRequest& input,
      const DraftQuarantinedReviewLabRequest& independently_verified_review) {
    const auto& req=input.http;
    const std::string prefix=std::string(kBackendMissionDraftPath)+
        "?mission_run_id=";
    if (req.path.compare(0,prefix.size(),prefix)!=0)
      return http_error(404,"UNKNOWN_API_ROUTE");
    if (req.method!="GET") return http_error(405,"METHOD_NOT_ALLOWED");
    if (!req.principal.authenticated)
      return http_error(401,"AUTHENTICATION_REQUIRED");
    if (!req.principal.can_read_mission)
      return http_error(403,"READ_SCOPE_REQUIRED");
    if (!req.body.empty()) return http_error(400,"BODY_NOT_ALLOWED");
    if (!std::isfinite(req.host_received_sec) || req.host_received_sec<0.0)
      return http_error(400,"INVALID_TRUSTED_RECEIVE_TIME");
    const auto id=req.path.substr(prefix.size());
    if (id.find_first_not_of(
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-")
        !=std::string::npos || !valid_native_mission_run_id(id))
      return http_error(400,"INVALID_DRAFT_QUERY");
    // A verified principal must be tied to the independent review identity.
    if (req.principal.operator_session_id!=
        independently_verified_review.current_identity.operator_session_id)
      return http_error(403,"REVIEW_IDENTITY_MISMATCH");
    BackendMissionUploadRequest typed;
    typed.method="GET";
    typed.path=kBackendMissionDraftPath;
    typed.principal=req.principal;
    typed.host_received_sec=req.host_received_sec;
    typed.mission_run_id=id;
    try {
      const auto reply=boot_.handle(typed,independently_verified_review);
      if (reply.http_status!=200)
        return http_error(reply.http_status,reply.code);
      // Metadata only: NEVER serialized marking coordinates or motion setpoints.
      const std::string body="{\"ok\":true,\"code\":"+
          http_json::quoted(reply.code)+",\"draft\":{\"revision\":"+
          std::to_string(reply.revision)+",\"signature\":"+
          http_json::quoted(reply.signature)+",\"marking_count\":"+
          std::to_string(reply.marking_count)+",\"navigation_count\":"+
          std::to_string(reply.navigation_count)+
          ",\"loaded\":false,\"preview_only\":true},"+
          "\"decision_only\":true,\"hardware_command_emitted\":false}";
      if(body.size()>4096) return http_error(500,"RESPONSE_TOO_LARGE");
      return {200,body};
    } catch (const std::exception&) {
      // Any database or encoding failure irreversibly closes PREVIEW access.
      retire();
      return http_error(423,"RECOVERY_REVIEW_FAILED");
    }
  }

  // Retiring the owner is irreversible. It does NOT release the durable fence
  // or the process-local gateway quarantine. Restart with a new owner only.
  void retire() { boot_.retire(); }

  bool has_active_mission() const { return gateway_.has_active_mission(); }
  bool has_live_backend_draft() const { return backend_.has_draft(); }
  bool emits_hardware_commands() const { return false; }

 private:
  static BackendHttpWireResponse http_error(int status,const std::string& code) {
    return {status,http_json::error_json(code)};
  }
  // Declaration order matters: gateway -> backend -> H5K guard.
  NativeMissionCommandGateway gateway_{};
  NativeBackendMissionUpload backend_;
  NativeOfflineQuarantinedBootGuardLab boot_;
  DraftQuarantinedBootGuardLabResult boot_admission_{};
};
} // namespace dyx::rpp
