#pragma once
// DYX Autonomous Phase 2H4O: OFFLINE Android upload/edit/update/LOAD UI contract.
// Neither transport nor actual Android code. UI booleans are suggestions, NEVER
// authentication, control leases or trusted PX4 disarm attestations. Do not
// serialize trusted backend fields from this proposal. Server MUST revalidate
// geometry, revision/signature, ownership, disarm and every command.

#include "dyx_rpp_cpp17_core/backend_mission_upload.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dyx::rpp {

enum class AndroidEditorIntent { upload, update, load };
inline const char* android_editor_route(AndroidEditorIntent i) {
  switch (i) {
    case AndroidEditorIntent::upload: return kBackendMissionUploadPath;
    case AndroidEditorIntent::update: return kBackendMissionUpdatePath;
    case AndroidEditorIntent::load: return kBackendMissionLoadPath;
  }
  return "";
}

struct AndroidEditorConfig {
  double max_draft_age_sec{2.0};
  double response_timeout_sec{2.0};
  std::size_t maximum_marking_points{10000};
  void validate() const {
    if (!std::isfinite(max_draft_age_sec) || max_draft_age_sec<0.1 ||
        max_draft_age_sec>30.0 || !std::isfinite(response_timeout_sec) ||
        response_timeout_sec<0.1 || response_timeout_sec>30.0 ||
        maximum_marking_points<2 || maximum_marking_points>10000)
      throw std::invalid_argument("invalid Android mission editor bounds");
  }
};

struct AndroidEditorScreenContext {
  std::string mission_run_id{};
  bool ui_signed_in{false};
  bool ui_control_lease_visible{false};
  bool ui_network_connected{false};
  bool ui_px4_disarmed_indicator{false};
  bool ui_confirmed_load{false};
  bool unsaved_local_changes{true};
  bool backend_draft_present{false};
  BackendMissionUploadResponse backend_draft{}; // externally authenticated view
  double backend_draft_received_sec{0.0};       // host monotonic observation
  std::vector<Point2D> marking_points{};       // LOCAL ENU metres, not WGS84
  bool extension_enabled{false};
  double extension_distance_m{4.0};
  MissionMode mission_mode{MissionMode::automatic};
};

struct AndroidEditorProposal {
  std::string code{"ANDROID_EDITOR_CONTEXT_REQUIRED"};
  std::string method{};
  std::string path{};
  std::string mission_run_id{};
  std::uint64_t sequence{0};
  std::uint64_t expected_revision{0};
  std::string expected_signature{};
  // UPLOAD/UPDATE contain typed local geometry; LOAD never carries points.
  // No client-supplied principal, lease, or trusted_px4_disarmed field.
  std::vector<Point2D> marking_points{};
  bool extension_enabled{false};
  double extension_distance_m{4.0};
  MissionMode mission_mode{MissionMode::automatic};
  bool prepared{false};
  bool backend_authorization_required{true};
  bool network_io_performed{false};
  bool hardware_command_emitted{false};
};

struct AndroidEditorAcknowledgment {
  std::string code{"ANDROID_EDITOR_NO_PENDING"};
  bool backend_decision_accepted{false};
  bool ready_to_load{false};
  bool backend_reports_loaded{false}; // not a motion or physical execution ACK
  std::uint64_t revision{0};
  std::string signature{};
  bool network_io_performed{false};
  bool hardware_command_emitted{false};
};

class NativeOfflineAndroidMissionEditor final {
 public:
  explicit NativeOfflineAndroidMissionEditor(AndroidEditorConfig cfg={})
      : cfg_(cfg) { cfg_.validate(); }

  // All screen fields are hints for hiding/enabling UI controls ONLY.
  // A new mission, sign-out, or disconnect fences an in-flight UI proposal.
  void show(AndroidEditorScreenContext screen) {
    const bool fence = have_screen_ &&
        (screen.mission_run_id != screen_.mission_run_id ||
         screen.ui_signed_in != screen_.ui_signed_in ||
         screen.ui_network_connected != screen_.ui_network_connected ||
         screen.ui_control_lease_visible != screen_.ui_control_lease_visible);
    if (fence || !screen.ui_network_connected) {
      pending_=false;
      requires_sync_=true;
    }
    screen_=std::move(screen);
    have_screen_=true;
  }

  AndroidEditorProposal propose(AndroidEditorIntent intent, double host_sec) {
    if (!have_screen_) return denied("ANDROID_EDITOR_CONTEXT_REQUIRED");
    if (!screen_.ui_signed_in) return denied("ANDROID_EDITOR_SIGN_IN_REQUIRED");
    if (!valid_native_mission_run_id(screen_.mission_run_id))
      return denied("ANDROID_EDITOR_MISSION_ID_REQUIRED");
    if (!screen_.ui_network_connected)
      return denied("ANDROID_EDITOR_NETWORK_UNAVAILABLE");
    if (!clock_ok(host_sec)) return denied("ANDROID_EDITOR_CLOCK_FAULT_LATCHED");
    if (sequence_fault_ || sequence_==std::numeric_limits<std::uint64_t>::max()) {
      sequence_fault_=true;
      return denied("ANDROID_EDITOR_SEQUENCE_EXHAUSTED");
    }
    if (requires_sync_) return denied("ANDROID_EDITOR_BACKEND_RESYNC_REQUIRED");
    if (pending_) return denied("ANDROID_EDITOR_REQUEST_PENDING");
    if (!screen_.ui_control_lease_visible)
      return denied("ANDROID_EDITOR_UI_LEASE_REQUIRED");
    if (!screen_.ui_px4_disarmed_indicator)
      return denied("ANDROID_EDITOR_UI_DISARM_REQUIRED");
    if (screen_.backend_draft_present && screen_.backend_draft.loaded)
      return denied("ANDROID_EDITOR_ALREADY_LOADED");

    const bool upload=intent==AndroidEditorIntent::upload;
    const bool load=intent==AndroidEditorIntent::load;
    if (upload && screen_.backend_draft_present)
      return denied("ANDROID_EDITOR_DRAFT_ALREADY_EXISTS");
    if (!upload && !fresh_draft(host_sec))
      return denied("ANDROID_EDITOR_FRESH_PREVIEW_REQUIRED");
    if (load) {
      if (screen_.unsaved_local_changes)
        return denied("ANDROID_EDITOR_UPDATE_BEFORE_LOAD");
      if (!screen_.ui_confirmed_load)
        return denied("ANDROID_EDITOR_LOAD_CONFIRMATION_REQUIRED");
      if (screen_.marking_points.size()!=screen_.backend_draft.marking_count)
        return denied("ANDROID_EDITOR_PREVIEW_COUNT_CONFLICT");
    } else {
      if (!screen_.unsaved_local_changes)
        return denied("ANDROID_EDITOR_NO_LOCAL_CHANGES");
      if (!valid_points() || !std::isfinite(screen_.extension_distance_m) ||
          screen_.extension_distance_m<0.10 || screen_.extension_distance_m>20.0 ||
          (screen_.mission_mode!=MissionMode::automatic &&
           screen_.mission_mode!=MissionMode::manual))
        return denied("ANDROID_EDITOR_INVALID_LOCAL_GEOMETRY");
    }
    const auto seq=++sequence_; // NEVER reuse even on timeout or denial
    pending_=true;
    pending_seq_=seq;
    pending_since_=host_sec;
    pending_run_=screen_.mission_run_id;
    pending_intent_=intent;
    pending_count_=screen_.marking_points.size();
    pending_revision_=upload?0:screen_.backend_draft.revision;
    pending_signature_=upload?std::string{}:screen_.backend_draft.signature;
    AndroidEditorProposal p;
    p.code="ANDROID_EDITOR_PROPOSAL_NOT_SENT";
    p.method="POST";
    p.path=android_editor_route(intent);
    p.mission_run_id=screen_.mission_run_id;
    p.sequence=seq;
    p.expected_revision=pending_revision_;
    p.expected_signature=pending_signature_;
    if (!load) {
      p.marking_points=screen_.marking_points;
      p.extension_enabled=screen_.extension_enabled;
      p.extension_distance_m=screen_.extension_distance_m;
      p.mission_mode=screen_.mission_mode;
    }
    p.prepared=true;
    return p;
  }

  // Only accept externally decoded, authenticated and sequence-correlated
  // backend results. This method performs extra consistency checks, but is NOT
  // a substitute for backend permission or proof of physical execution.
  AndroidEditorAcknowledgment observe_backend(std::uint64_t request_seq,
      const BackendMissionUploadResponse& response, double host_sec) {
    if (!pending_ || request_seq!=pending_seq_ || !have_screen_ ||
        screen_.mission_run_id!=pending_run_)
      return ack("ANDROID_EDITOR_REPLY_UNMATCHED_OR_REPLAYED");
    if (!clock_ok(host_sec)) {
      pending_=false;
      requires_sync_=true;
      return ack("ANDROID_EDITOR_CLOCK_FAULT_LATCHED");
    }
    if (host_sec-pending_since_>cfg_.response_timeout_sec) {
      pending_=false;
      requires_sync_=true;
      return ack("ANDROID_EDITOR_RESULT_UNKNOWN_RESYNC");
    }
    pending_=false;
    requires_sync_=true; // always GET fresh preview before another mutation
    const bool upload=pending_intent_==AndroidEditorIntent::upload;
    const bool update=pending_intent_==AndroidEditorIntent::update;
    const bool load=pending_intent_==AndroidEditorIntent::load;
    const char* expected=upload?"UPLOAD_PREVIEW_READY":
                         update?"UPDATED_PREVIEW_READY":"MISSION_LOADED_READY_NOT_STARTED";
    if (response.hardware_command_emitted || response.http_status!=200 ||
        response.code!=expected || !response.draft_ready ||
        response.loaded!=load || response.preview_only==load ||
        !valid_path_signature(response.signature) ||
        response.marking_count!=pending_count_ ||
        response.navigation_count<response.marking_count ||
        response.revision!=(upload?1:(update?pending_revision_+1:pending_revision_)) ||
        (load && response.signature!=pending_signature_) ||
        (update && pending_revision_==std::numeric_limits<std::uint64_t>::max()))
      return ack("ANDROID_EDITOR_DENIED_OR_INCONSISTENT_RESYNC");
    AndroidEditorAcknowledgment a;
    a.code=load?"ANDROID_EDITOR_LOAD_REPORTED_NOT_STARTED":"ANDROID_EDITOR_PREVIEW_REPORTED";
    a.backend_decision_accepted=true;
    a.ready_to_load=!load;
    a.backend_reports_loaded=load;
    a.revision=response.revision;
    a.signature=response.signature;
    return a;
  }

  AndroidEditorAcknowledgment expire(double host_sec) {
    if (!clock_ok(host_sec)) {
      pending_=false;requires_sync_=true;
      return ack("ANDROID_EDITOR_CLOCK_FAULT_LATCHED");
    }
    if (!pending_) return ack("ANDROID_EDITOR_NO_PENDING");
    if (host_sec-pending_since_>cfg_.response_timeout_sec) {
      pending_=false;requires_sync_=true;
      return ack("ANDROID_EDITOR_TIMEOUT_UNKNOWN_RESYNC");
    }
    return ack("ANDROID_EDITOR_WAITING_FOR_BACKEND");
  }

  // Fresh GET /draft after every mutation, denial, loss or timeout. This is
  // advisory sync only; the real backend re-authorizes mutations separately.
  bool synced_from_backend(const BackendMissionUploadResponse& fresh,
                           double host_received_sec, double host_now_sec) {
    if (!have_screen_ || !screen_.ui_signed_in || !screen_.ui_network_connected ||
        !clock_ok(host_now_sec) || !std::isfinite(host_received_sec) ||
        host_received_sec<0.0 || host_received_sec>host_now_sec ||
        host_now_sec-host_received_sec>cfg_.max_draft_age_sec || pending_) return false;
    if (fresh.http_status!=200 || !fresh.draft_ready ||
        !valid_path_signature(fresh.signature) || fresh.revision==0 ||
        fresh.marking_count<2 || fresh.navigation_count<fresh.marking_count ||
        fresh.hardware_command_emitted ||
        (fresh.loaded && fresh.preview_only) ||
        (!fresh.loaded && !fresh.preview_only)) return false;
    screen_.backend_draft=fresh;
    screen_.backend_draft_present=true;
    screen_.backend_draft_received_sec=host_received_sec;
    requires_sync_=false;
    return true;
  }
  // For first UPLOAD only, a trusted current GET /draft returning 404 may
  // confirm no draft (not proof that an arbitrary client can upload).
  bool synced_empty_draft(bool verified_backend_404, double host_sec) {
    if (!have_screen_ || !screen_.ui_signed_in || !screen_.ui_network_connected ||
        !verified_backend_404 || pending_ || !clock_ok(host_sec)) return false;
    screen_.backend_draft_present=false;
    screen_.backend_draft={};
    requires_sync_=false;
    return true;
  }
  // Any undecidable outcome (codec failure, lost reply, untrusted response)
  // must clear the pending UI proposal and require an authoritative GET.
  // This never accepts an action, starts a mission or resets its sequence.
  void abandon_pending_as_unknown() { pending_=false; requires_sync_=true; }

  void disconnected() {
    pending_=false;requires_sync_=true;
    screen_.ui_network_connected=false;
  }
  bool recover_disarmed(bool independently_verified_operator,
                        bool independently_verified_disarmed, double new_host_sec) {
    if (!independently_verified_operator || !independently_verified_disarmed ||
        !std::isfinite(new_host_sec) || new_host_sec<0.0 || sequence_fault_) return false;
    clock_fault_=false;last_clock_=new_host_sec;pending_=false;
    have_screen_=false;requires_sync_=true;screen_={};
    return true; // sequence deliberately not reset
  }
  bool has_active_screen_for_mission(const std::string& mission_run_id) const {
    return have_screen_ && screen_.mission_run_id == mission_run_id;
  }
  bool pending() const { return pending_; }
  bool requires_resync() const { return requires_sync_; }
  bool clock_fault_latched() const { return clock_fault_; }
  std::uint64_t latest_sequence() const { return sequence_; }
 private:
  static AndroidEditorProposal denied(const char* code) {
    AndroidEditorProposal p;p.code=code;return p;
  }
  static AndroidEditorAcknowledgment ack(const char* code) {
    AndroidEditorAcknowledgment a;a.code=code;return a;
  }
  bool clock_ok(double t) {
    if (clock_fault_) return false;
    if (!std::isfinite(t) || t<0.0 || t<last_clock_) {
      clock_fault_=true;pending_=false;requires_sync_=true;return false;
    }
    last_clock_=t;return true;
  }
  bool fresh_draft(double now) const {
    const auto& b=screen_.backend_draft;
    return screen_.backend_draft_present &&
        std::isfinite(screen_.backend_draft_received_sec) &&
        screen_.backend_draft_received_sec>=0.0 &&
        screen_.backend_draft_received_sec<=now &&
        now-screen_.backend_draft_received_sec<=cfg_.max_draft_age_sec &&
        b.http_status==200 && b.draft_ready && !b.loaded && b.preview_only &&
        b.revision>0 && valid_path_signature(b.signature) &&
        b.marking_count>=2 && b.navigation_count>=b.marking_count &&
        !b.hardware_command_emitted;
  }
  bool valid_points() const {
    const auto& points=screen_.marking_points;
    if (points.size()<2 || points.size()>cfg_.maximum_marking_points) return false;
    for (std::size_t i=0;i<points.size();++i) {
      if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y) ||
          std::abs(points[i].x)>10000.0 || std::abs(points[i].y)>10000.0) return false;
      if (i && (!std::isfinite(std::hypot(points[i].x-points[i-1].x,
                                          points[i].y-points[i-1].y)) ||
                std::hypot(points[i].x-points[i-1].x,
                           points[i].y-points[i-1].y)<0.001)) return false;
    }
    return true;
  }
  AndroidEditorConfig cfg_{};
  AndroidEditorScreenContext screen_{};
  bool have_screen_{false};
  bool pending_{false};
  bool requires_sync_{true};
  bool clock_fault_{false};
  bool sequence_fault_{false};
  std::uint64_t sequence_{0},pending_seq_{0},pending_revision_{0};
  double last_clock_{0.0},pending_since_{0.0};
  std::string pending_run_{},pending_signature_{};
  std::size_t pending_count_{0};
  AndroidEditorIntent pending_intent_{AndroidEditorIntent::upload};
};

} // namespace dyx::rpp
