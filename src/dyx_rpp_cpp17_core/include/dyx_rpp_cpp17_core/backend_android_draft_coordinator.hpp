#pragma once
// DYX Autonomous Phase 2H4S: OFFLINE coordinator of an Android mission-editor
// proposal, strict HTTP/JSON codec, and trusted draft GET reconciliation.
// The caller provides independently authenticated transport evidence; no UI
// boolean, JSON field or coordinator method proves identity or PX4 disarm.
// This class owns no sockets, ROS 2, DDS, PX4, motor, spray or laser interfaces.

#include "dyx_rpp_cpp17_core/backend_android_draft_sync.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace dyx::rpp {

struct AndroidDraftCoordinatorConfig {
  AndroidEditorConfig editor{};
  AndroidDraftHttpCodecConfig codec{};
  AndroidDraftSyncConfig sync{};
  double mutation_total_timeout_sec{2.0};
  void validate() const {
    editor.validate();
    codec.validate();
    sync.validate();
    if (!std::isfinite(mutation_total_timeout_sec) ||
        mutation_total_timeout_sec < 0.1 || mutation_total_timeout_sec > 30.0)
      throw std::invalid_argument("invalid Android draft transaction timeout");
  }
};

// A result concerns the BACKEND decision only. It is never an actuator ACK.
struct AndroidDraftCoordinatorResult {
  bool verified_backend_decision{false};
  bool requires_authoritative_resync{true};
  bool loaded_ready_not_started{false};
  std::string code{"ANDROID_COORDINATOR_REPLY_REJECTED"};
  bool network_io_performed{false};
  bool hardware_command_emitted{false};
};

class NativeOfflineAndroidDraftCoordinator final {
 public:
  explicit NativeOfflineAndroidDraftCoordinator(AndroidDraftCoordinatorConfig cfg={})
      : cfg_(cfg), editor_(cfg.editor), codec_(cfg.codec), sync_(cfg.sync) {
    cfg_.validate();
  }

  // The screen is UNTRUSTED: never accept cached backend revision/signature
  // from it. Only a verified GET /draft performed by this coordinator may
  // populate those fields for the embedded editor.
  void show(AndroidEditorScreenContext untrusted) {
    const bool changed = !have_view_ ||
        untrusted.mission_run_id != view_run_ ||
        untrusted.ui_signed_in != signed_in_ ||
        untrusted.ui_network_connected != network_connected_ ||
        untrusted.ui_control_lease_visible != lease_visible_;
    if (changed) {
      mutation_pending_=false;
      pending_mutation_={};
      sync_.invalidate();
      cached_draft_present_=false;
      cached_draft_={};
    }
    view_run_=untrusted.mission_run_id;
    signed_in_=untrusted.ui_signed_in;
    network_connected_=untrusted.ui_network_connected;
    lease_visible_=untrusted.ui_control_lease_visible;
    have_view_=true;
    untrusted.backend_draft_present=cached_draft_present_;
    untrusted.backend_draft=cached_draft_;
    untrusted.backend_draft_received_sec=cached_draft_received_sec_;
    editor_.show(std::move(untrusted));
  }

  AndroidDraftEncodedRequest begin_mutation(AndroidEditorIntent intent,
                                             double host_now_sec) {
    if (!check_clock(host_now_sec)) return denied("ANDROID_COORDINATOR_CLOCK_FAULT");
    if (mutation_pending_ || sync_.pending())
      return denied("ANDROID_COORDINATOR_ANOTHER_REQUEST_PENDING");
    const auto proposal=editor_.propose(intent,host_now_sec);
    if (!proposal.prepared) return denied(proposal.code.c_str());
    const auto encoded=codec_.encode(proposal);
    if (!encoded.valid) {
      editor_.abandon_pending_as_unknown();
      cached_draft_present_=false;
      return denied("ANDROID_COORDINATOR_ENCODING_FAILED_RESYNC");
    }
    pending_mutation_=encoded;
    mutation_pending_=true;
    mutation_started_sec_=host_now_sec;
    return encoded; // PREPARED, not transmitted
  }

  AndroidDraftCoordinatorResult complete_mutation(
      const AndroidDraftEncodedRequest& original,
      const AndroidTrustedDraftHttpReply& trusted,
      double host_now_sec) {
    if (!check_clock(host_now_sec)) return denied_result("ANDROID_COORDINATOR_CLOCK_FAULT");
    if (!mutation_pending_) return denied_result("ANDROID_COORDINATOR_NO_PENDING_MUTATION");
    const auto expected=pending_mutation_;
    mutation_pending_=false;
    pending_mutation_={}; // even forged replies consume the flight
    if (!original.valid || !original.body.size() ||
        original.method != expected.method || original.path != expected.path ||
        original.body != expected.body || original.sequence != expected.sequence ||
        original.network_io_performed || original.hardware_command_emitted ||
        !trusted.verified_authenticated_transport ||
        !std::isfinite(trusted.verified_sent_sec) ||
        !std::isfinite(trusted.verified_received_sec) ||
        trusted.verified_sent_sec < mutation_started_sec_ ||
        trusted.verified_received_sec > host_now_sec ||
        host_now_sec-mutation_started_sec_ > cfg_.mutation_total_timeout_sec ||
        host_now_sec-trusted.verified_received_sec > cfg_.sync.max_reply_age_sec ||
        trusted.wire.network_io_performed || trusted.wire.physical_command_emitted) {
      editor_.abandon_pending_as_unknown();
      invalidate_cached_draft();
      return denied_result("ANDROID_COORDINATOR_UNTRUSTED_RESULT_RESYNC");
    }
    const auto decoded=codec_.decode(trusted,expected);
    if (!decoded.valid) {
      editor_.abandon_pending_as_unknown();
      invalidate_cached_draft();
      return denied_result("ANDROID_COORDINATOR_DECODE_DENIED_RESYNC");
    }
    const auto ack=editor_.observe_backend(expected.sequence,decoded.backend,host_now_sec);
    // A POST response NEVER clears the resync fence; only verified GET does.
    invalidate_cached_draft();
    AndroidDraftCoordinatorResult result;
    result.verified_backend_decision=ack.backend_decision_accepted;
    result.loaded_ready_not_started=ack.backend_reports_loaded &&
        ack.backend_decision_accepted;
    result.requires_authoritative_resync=true;
    result.code=ack.code;
    return result;
  }

  AndroidDraftCoordinatorResult expire_mutation(double host_now_sec) {
    if (!check_clock(host_now_sec)) return denied_result("ANDROID_COORDINATOR_CLOCK_FAULT");
    if (!mutation_pending_) return denied_result("ANDROID_COORDINATOR_NO_PENDING_MUTATION");
    if (host_now_sec-mutation_started_sec_ <= cfg_.mutation_total_timeout_sec)
      return denied_result("ANDROID_COORDINATOR_MUTATION_STILL_PENDING");
    mutation_pending_=false;
    pending_mutation_={};
    editor_.abandon_pending_as_unknown();
    invalidate_cached_draft();
    return denied_result("ANDROID_COORDINATOR_RESULT_UNKNOWN_RESYNC");
  }

  AndroidDraftSyncRequest begin_resync(double host_now_sec) {
    if (!check_clock(host_now_sec) || mutation_pending_ || !have_view_ ||
        !signed_in_ || !network_connected_ || !valid_native_mission_run_id(view_run_))
      return {};
    return sync_.begin(view_run_,host_now_sec); // GET prepared, not sent
  }

  AndroidDraftSyncDecision complete_resync(
      const AndroidDraftSyncRequest& request,
      const AndroidTrustedDraftSyncReply& reply,double host_now_sec) {
    if (!check_clock(host_now_sec)) return {};
    auto result=sync_.complete(request,reply,host_now_sec,editor_);
    if (result.verified && result.editor_resynchronized) {
      cached_draft_present_=result.draft_present;
      cached_draft_=result.draft_present?result.draft:BackendMissionUploadResponse{};
      cached_draft_received_sec_=reply.verified_received_sec;
    } else {
      invalidate_cached_draft();
      editor_.abandon_pending_as_unknown();
    }
    return result;
  }

  void resync_lost() {
    sync_.invalidate();
    editor_.abandon_pending_as_unknown();
    invalidate_cached_draft();
  }

  bool mutation_pending() const { return mutation_pending_; }
  bool sync_pending() const { return sync_.pending(); }
  bool requires_resync() const { return editor_.requires_resync(); }
  bool clock_fault_latched() const { return clock_fault_; }
  std::uint64_t last_mutation_sequence() const { return editor_.latest_sequence(); }
 private:
  static AndroidDraftEncodedRequest denied(const char* why) {
    AndroidDraftEncodedRequest out;
    out.code=why;
    return out;
  }
  static AndroidDraftCoordinatorResult denied_result(const char* why) {
    AndroidDraftCoordinatorResult out;
    out.code=why;
    return out;
  }
  void invalidate_cached_draft() {
    cached_draft_present_=false;
    cached_draft_={};
    cached_draft_received_sec_=0.0;
  }
  bool check_clock(double now) {
    if (clock_fault_) return false;
    if (!std::isfinite(now) || now<0.0 || now<last_clock_sec_) {
      clock_fault_=true;
      mutation_pending_=false;
      pending_mutation_={};
      sync_.invalidate();
      editor_.abandon_pending_as_unknown();
      invalidate_cached_draft();
      return false;
    }
    last_clock_sec_=now;
    return true;
  }
  AndroidDraftCoordinatorConfig cfg_{};
  NativeOfflineAndroidMissionEditor editor_{};
  NativeOfflineAndroidDraftHttpCodec codec_{};
  NativeOfflineAndroidDraftSync sync_{};
  AndroidDraftEncodedRequest pending_mutation_{};
  bool mutation_pending_{false};
  double mutation_started_sec_{0.0};
  bool have_view_{false};
  bool signed_in_{false};
  bool network_connected_{false};
  bool lease_visible_{false};
  std::string view_run_{};
  bool cached_draft_present_{false};
  BackendMissionUploadResponse cached_draft_{};
  double cached_draft_received_sec_{0.0};
  bool clock_fault_{false};
  double last_clock_sec_{0.0};
};

} // namespace dyx::rpp
