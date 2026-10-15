#pragma once
// DYX Autonomous Phase 2H4R: OFFLINE, single-flight Android GET /draft
// reconciliation. This is not an HTTP client, identity verifier, TLS stack,
// Jetson service or actuator control path. Trusted reply metadata is supplied
// independently by a future privileged transport, NEVER by the UI or JSON.

#include "dyx_rpp_cpp17_core/backend_android_draft_http_codec.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace dyx::rpp {

struct AndroidDraftSyncConfig {
  std::size_t max_response_bytes{4096};
  double max_in_flight_sec{2.0};
  double max_reply_age_sec{2.0};
  void validate() const {
    if (max_response_bytes < 512 || max_response_bytes > 65536 ||
        !std::isfinite(max_in_flight_sec) || max_in_flight_sec < 0.1 ||
        max_in_flight_sec > 30.0 || !std::isfinite(max_reply_age_sec) ||
        max_reply_age_sec < 0.1 || max_reply_age_sec > 30.0)
      throw std::invalid_argument("invalid Android draft sync limits");
  }
};

struct AndroidDraftSyncRequest {
  bool valid{false};
  std::string method{};
  std::string path{};
  std::string body{};
  std::string mission_run_id{};
  std::uint64_t request_id{0};
  bool network_io_performed{false};
  bool hardware_command_emitted{false};
};

struct AndroidTrustedDraftSyncReply {
  BackendHttpWireResponse wire{};
  bool verified_authenticated_transport{false};
  std::uint64_t verified_request_id{0};
  std::string verified_request_path{};
  double verified_sent_sec{std::numeric_limits<double>::quiet_NaN()};
  double verified_received_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct AndroidDraftSyncDecision {
  bool verified{false};
  bool editor_resynchronized{false};
  bool draft_present{false};
  std::string code{"ANDROID_DRAFT_RESYNC_DENIED"};
  BackendMissionUploadResponse draft{};
  bool network_io_performed{false};
  bool hardware_command_emitted{false};
};

// A call to begin() only PREPARES a request. The application/transport
// must send it separately and cryptographically verify reply provenance.
// No network, PX4, DDS or physical I/O is performed by this class.
class NativeOfflineAndroidDraftSync final {
 public:
  explicit NativeOfflineAndroidDraftSync(AndroidDraftSyncConfig cfg={})
      : cfg_(cfg) { cfg_.validate(); }

  AndroidDraftSyncRequest begin(const std::string& mission_run_id,
                                double host_now_sec) {
    if (!clock_ok(host_now_sec) || outstanding_ ||
        request_id_ == std::numeric_limits<std::uint64_t>::max() ||
        !valid_native_mission_run_id(mission_run_id)) return {};
    AndroidDraftSyncRequest request;
    request.valid = true;
    request.method = "GET";
    request.mission_run_id = mission_run_id;
    request.path = std::string(kBackendMissionDraftPath) +
        "?mission_run_id=" + mission_run_id;
    request.request_id = ++request_id_;
    outstanding_ = true;
    started_sec_ = host_now_sec;
    pending_ = request;
    return request;
  }

  // A bad, delayed or forged reply consumes the in-flight request. It must
  // never authorize another mutation. The UI remains in resync-required mode.
  AndroidDraftSyncDecision complete(const AndroidDraftSyncRequest& request,
       const AndroidTrustedDraftSyncReply& verified_reply, double host_now_sec,
       NativeOfflineAndroidMissionEditor& editor) {
    AndroidDraftSyncDecision decision;
    if (!outstanding_) return decision;
    const auto expected = pending_;
    outstanding_ = false;
    pending_ = {};
    if (!clock_ok(host_now_sec) || !request.valid ||
        request.method != "GET" || !request.body.empty() ||
        request.request_id != expected.request_id ||
        request.mission_run_id != expected.mission_run_id ||
        request.path != expected.path ||
        !editor.has_active_screen_for_mission(expected.mission_run_id) ||
        !verified_reply.verified_authenticated_transport ||
        verified_reply.verified_request_id != expected.request_id ||
        verified_reply.verified_request_path != expected.path ||
        !std::isfinite(verified_reply.verified_sent_sec) ||
        !std::isfinite(verified_reply.verified_received_sec) ||
        verified_reply.verified_sent_sec < started_sec_ ||
        verified_reply.verified_received_sec < verified_reply.verified_sent_sec ||
        verified_reply.verified_received_sec > host_now_sec ||
        host_now_sec - started_sec_ > cfg_.max_in_flight_sec ||
        host_now_sec - verified_reply.verified_received_sec > cfg_.max_reply_age_sec ||
        verified_reply.wire.content_type != "application/json; charset=utf-8" ||
        verified_reply.wire.body.size() > cfg_.max_response_bytes ||
        verified_reply.wire.physical_command_emitted ||
        verified_reply.wire.network_io_performed)
      return decision;
    const auto& wire = verified_reply.wire;
    if (wire.status == 404) {
      // Only the EXACT server error may establish an empty draft. Other 404s
      // (wrong route or authorization ambiguity) must never clear the UI.
      if (wire.body != "{\"ok\":false,\"code\":\"NO_PENDING_DRAFT\"}")
        return decision;
      decision.verified = true;
      decision.editor_resynchronized = editor.synced_empty_draft(true, host_now_sec);
      if (!decision.editor_resynchronized) return decision;
      decision.code = "ANDROID_DRAFT_CONFIRMED_ABSENT";
      return decision;
    }
    if (wire.status != 200) return decision;
    BackendMissionUploadResponse draft;
    if (!android_draft_detail::StrictDraftSuccessReader(wire.body).parse(draft))
      return decision;
    if (!valid_path_signature(draft.signature) || draft.revision == 0 ||
        draft.marking_count < 2 ||
        draft.navigation_count < draft.marking_count ||
        draft.dummy_count > draft.navigation_count ||
        !draft.draft_ready || draft.preview_only == draft.loaded ||
        draft.code != (draft.loaded ? "MISSION_LOADED" : "DRAFT_PREVIEW_READY"))
      return decision;
    draft.http_status = 200;
    draft.hardware_command_emitted = false;
    decision.verified = true;
    decision.draft_present = true;
    decision.draft = draft;
    decision.editor_resynchronized = editor.synced_from_backend(
        draft, verified_reply.verified_received_sec, host_now_sec);
    if (decision.editor_resynchronized)
      decision.code = draft.loaded ? "ANDROID_DRAFT_LOADED_SYNCED_NOT_STARTED" :
          "ANDROID_DRAFT_PREVIEW_SYNCED";
    return decision;
  }

  void invalidate() { outstanding_ = false; pending_ = {}; }
  bool pending() const { return outstanding_; }
  bool clock_fault_latched() const { return clock_fault_; }
  std::uint64_t last_request_id() const { return request_id_; }
 private:
  bool clock_ok(double now) {
    if (clock_fault_) return false;
    if (!std::isfinite(now) || now < 0.0 || now < last_clock_) {
      clock_fault_ = true;
      invalidate();
      return false;
    }
    last_clock_ = now;
    return true;
  }
  AndroidDraftSyncConfig cfg_{};
  AndroidDraftSyncRequest pending_{};
  bool outstanding_{false};
  bool clock_fault_{false};
  double started_sec_{0.0};
  double last_clock_{0.0};
  std::uint64_t request_id_{0};
};

} // namespace dyx::rpp
