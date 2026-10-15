#pragma once
// Phase 2H4E: Windows-testable, transport-independent UPLOAD -> optional UPDATE
// -> explicit LOAD boundary. No HTTP listener, ROS, PX4 transport or actuation.
// Coordinates are local ENU metres, not WGS84 lat/lon. A digest protects
// snapshot CONSISTENCY and is not authentication. Authorization/disarm facts
// MUST be supplied by a privileged, independent backend adapter, never JSON.

#include "dyx_rpp_cpp17_core/backend_mission_api.hpp"
#include "dyx_rpp_cpp17_core/mission_runtime_install.hpp"
#include "dyx_rpp_cpp17_core/trajectory_builder.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace dyx::rpp {

constexpr const char *kBackendMissionUploadPath = "/api/v1/mission/upload";
constexpr const char *kBackendMissionUpdatePath = "/api/v1/mission/update";
constexpr const char *kBackendMissionLoadPath = "/api/v1/mission/load";
constexpr const char *kBackendMissionDraftPath = "/api/v1/mission/draft";

struct BackendMissionUploadRequest {
  std::string method{"POST"};
  std::string path{};
  BackendVerifiedPrincipal principal{};  // server-verified principal only
  std::string mission_run_id{};           // unique, bound to this draft
  std::uint64_t sequence{0};              // per-draft monotonic mutation sequence
  double host_received_sec{std::numeric_limits<double>::quiet_NaN()};
  // Required for UPDATE and LOAD. Prevents stale edits and accidental LOAD.
  std::uint64_t expected_revision{0};
  std::string expected_signature{};
  // Only UPLOAD/UPDATE accept local ENU points. LOAD does not accept geometry.
  std::vector<Point2D> marking_points{};
  bool extension_enabled{false};
  double extension_distance_m{4.0};
  MissionMode mission_mode{MissionMode::automatic};
  // Trusted PX4 disarm attestation from the backend, NOT from request JSON.
  bool trusted_px4_disarmed{false};
};

struct BackendMissionUploadResponse {
  int http_status{500};
  std::string code{"INTERNAL_ERROR"};
  std::uint64_t revision{0};
  std::string signature{};
  std::size_t marking_count{0};
  std::size_t navigation_count{0};
  std::size_t dummy_count{0};
  bool draft_ready{false};
  bool loaded{false};
  bool preview_only{true};
  bool hardware_command_emitted{false};   // invariant false
  bool accepted() const { return http_status == 200; }
};

class NativeBackendMissionUpload final {
 public:
  explicit NativeBackendMissionUpload(NativeMissionCommandGateway &gateway)
      : gateway_(gateway) {}

  // One-way latch shared with every upload backend using the same gateway.
  void latch_recovery_quarantine() { gateway_.latch_recovery_quarantine(); }
  bool recovery_quarantined() const { return gateway_.recovery_quarantined(); }

  BackendMissionUploadResponse handle(const BackendMissionUploadRequest &req) {
    if (gateway_.recovery_quarantined())
      return reply(423, "RECOVERY_QUARANTINE_COMMANDS_LOCKED");
    const bool upload = req.path == kBackendMissionUploadPath;
    const bool update = req.path == kBackendMissionUpdatePath;
    const bool load = req.path == kBackendMissionLoadPath;
    const bool preview = req.path == kBackendMissionDraftPath;
    if (!upload && !update && !load && !preview)
      return reply(404, "UNKNOWN_API_ROUTE");
    if (req.method != (preview ? "GET" : "POST"))
      return reply(405, "METHOD_NOT_ALLOWED");
    if (!req.principal.authenticated)
      return reply(401, "AUTHENTICATION_REQUIRED");
    if (preview) {
      if (!req.principal.can_read_mission)
        return reply(403, "READ_SCOPE_REQUIRED");
      if (!draft_) return reply(404, "NO_PENDING_DRAFT");
      if (req.principal.operator_session_id != draft_->operator_session_id ||
          req.mission_run_id != draft_->run_id)
        return reply(403, "DRAFT_OWNER_REQUIRED");
      return reply(200, draft_->loaded ? "MISSION_LOADED" : "DRAFT_PREVIEW_READY");
    }
    if (!req.principal.owns_control_lease)
      return reply(403, "CONTROL_LEASE_REQUIRED");
    if (!valid_native_mission_run_id(req.mission_run_id) ||
        !valid_native_mission_run_id(req.principal.operator_session_id) ||
        req.sequence == 0 || !std::isfinite(req.host_received_sec) ||
        req.host_received_sec < 0.0)
      return reply(400, "INVALID_COMMAND_ENVELOPE");
    if (draft_) {
      if (req.mission_run_id != draft_->run_id ||
          req.principal.operator_session_id != draft_->operator_session_id)
        return reply(409, "DRAFT_IDENTITY_CONFLICT");
      if (req.sequence <= draft_->highest_sequence)
        return reply(409, "COMMAND_REPLAY_REJECTED");
      if (req.host_received_sec < draft_->last_command_time_sec)
        return reply(400, "NON_MONOTONIC_HOST_TIME");
      if (draft_->loaded || gateway_.has_active_mission())
        return reply(409, "MISSION_ALREADY_LOADED");
      // Consume authenticated, ordered commands even when they are rejected.
      // A previously unsafe edit/load can never become valid by replay.
      draft_->highest_sequence = req.sequence;
      draft_->last_command_time_sec = req.host_received_sec;
    } else if (!upload) {
      return reply(409, "UPLOAD_REQUIRED_BEFORE_LOAD");
    }
    if (upload && draft_) return reply(409, "DRAFT_ALREADY_EXISTS");
    if (load) return commit_load(req);
    return commit_draft(req, upload);
  }

  // Explicit authorized discard is only possible BEFORE LOAD. Cancel never
  // stops an active rover mission: use authenticated STOP and PX4 safe-stop.
  bool discard_unloaded(bool verified_operator, bool verified_disarmed) {
    if (gateway_.recovery_quarantined()) return false;
    if (!verified_operator || !verified_disarmed || !draft_ || draft_->loaded)
      return false;
    draft_.reset();
    return true;
  }
  bool has_draft() const { return draft_.has_value(); }
  bool loaded() const { return draft_ && draft_->loaded; }
  const PreparedPathSnapshot *preview_snapshot() const {
    if (gateway_.recovery_quarantined()) return nullptr;
    return draft_ ? &draft_->snapshot : nullptr;  // immutable read-only preview
  }

 private:
  struct Draft {
    std::string run_id;
    std::string operator_session_id;
    PreparedPathSnapshot snapshot;
    MissionMode mode{MissionMode::automatic};
    std::uint64_t revision{1}, highest_sequence{0};
    double last_command_time_sec{0.0};
    std::size_t dummy_count{0};
    bool loaded{false};
  };

  BackendMissionUploadResponse commit_draft(const BackendMissionUploadRequest &req,
                                           bool initial_upload) {
    if (gateway_.has_active_mission())
      return reply(409, "MISSION_ALREADY_LOADED");
    if (!req.trusted_px4_disarmed)
      return reply(412, "PX4_DISARM_REQUIRED");
    if (!initial_upload &&
        (req.expected_revision != draft_->revision ||
         req.expected_signature != draft_->snapshot.signature))
      return reply(409, "DRAFT_REVISION_CONFLICT");
    if (!initial_upload && draft_->revision == std::numeric_limits<std::uint64_t>::max())
      return reply(409, "DRAFT_REVISION_EXHAUSTED");
    // UI extension distance accepts 0.10–20.0 m; trigger remains <3 m.
    if (req.mission_mode != MissionMode::automatic &&
        req.mission_mode != MissionMode::manual)
      return reply(422, "INVALID_MISSION_MODE");
    if (!std::isfinite(req.extension_distance_m) ||
        req.extension_distance_m < 0.10 || req.extension_distance_m > 20.0)
      return reply(422, "INVALID_EXTENSION_DISTANCE");
    PreparedPathSnapshot snapshot;
    std::size_t dummy_count = 0;
    try {
      TrajectoryBuildConfig cfg;
      cfg.extension_enabled = req.extension_enabled;
      cfg.extension_distance_m = req.extension_distance_m;
      const auto trajectory = NativeTrajectoryBuilder(cfg).build(req.marking_points);
      snapshot = signed_snapshot(trajectory.navigation, req.marking_points);
      dummy_count = trajectory.dummy_count;
      // Check exact downstream snapshot binding BEFORE publishing the draft.
      const NativeMissionInstallation validated(snapshot, req.mission_run_id,
                                                req.mission_mode);
      if (validated.marking_goals.size() != req.marking_points.size())
        return reply(422, "INVALID_PREPARED_PATH");
    } catch (const std::exception &) {
      return reply(422, "INVALID_MISSION_GEOMETRY");
    }
    if (initial_upload) {
      Draft d;
      d.run_id = req.mission_run_id;
      d.operator_session_id = req.principal.operator_session_id;
      d.highest_sequence = req.sequence;
      d.last_command_time_sec = req.host_received_sec;
      d.snapshot = std::move(snapshot);
      d.mode = req.mission_mode;
      d.dummy_count = dummy_count;
      draft_ = std::move(d);
      return reply(200, "UPLOAD_PREVIEW_READY");
    }
    // Transactional replacement: a failed edit NEVER destroys old preview.
    draft_->snapshot = std::move(snapshot);
    draft_->mode = req.mission_mode;
    draft_->dummy_count = dummy_count;
    ++draft_->revision;
    return reply(200, "UPDATED_PREVIEW_READY");
  }

  BackendMissionUploadResponse commit_load(const BackendMissionUploadRequest &req) {
    if (!req.trusted_px4_disarmed)
      return reply(412, "PX4_DISARM_REQUIRED");
    if (req.expected_revision != draft_->revision ||
        req.expected_signature != draft_->snapshot.signature)
      return reply(409, "DRAFT_REVISION_CONFLICT");
    if (!req.marking_points.empty())
      return reply(400, "LOAD_MUST_USE_SAVED_PREVIEW");
    try {
      // Revalidate the COMPLETE immutable snapshot at the load boundary.
      NativeMissionInstallation installed(draft_->snapshot, draft_->run_id,
                                          draft_->mode);
      const auto status = gateway_.load(
          draft_->run_id, draft_->operator_session_id,
          std::move(installed.marking_goals), draft_->mode,
          true, req.principal.owns_control_lease, req.trusted_px4_disarmed);
      switch (status) {
        case ClientCommandStatus::accepted:
          draft_->loaded = true;
          return reply(200, "MISSION_LOADED_READY_NOT_STARTED");
        case ClientCommandStatus::run_id_reused:
          return reply(409, "MISSION_RUN_ID_REUSED");
        case ClientCommandStatus::unauthorized:
          return reply(403, "LOAD_NOT_AUTHORIZED");
        case ClientCommandStatus::invalid_stage:
          return reply(409, "MISSION_ALREADY_LOADED");
        default:
          return reply(412, "MISSION_RUNTIME_LOAD_REJECTED");
      }
    } catch (const std::exception &) {
      return reply(422, "INVALID_PREPARED_PATH");
    }
  }

  BackendMissionUploadResponse reply(int http_status, const char *code) const {
    BackendMissionUploadResponse r;
    r.http_status = http_status;
    r.code = code;
    // Never disclose another operator's draft on denied/error responses.
    // Authorized clients can retrieve the current revision via GET /draft.
    if (draft_ && http_status == 200) {
      r.revision = draft_->revision;
      r.signature = draft_->snapshot.signature;
      r.marking_count = draft_->snapshot.marking_points.size();
      r.navigation_count = draft_->snapshot.navigation_points.size();
      r.dummy_count = draft_->dummy_count;
      r.draft_ready = true;
      r.loaded = draft_->loaded;
      r.preview_only = !r.loaded;
    }
    return r;
  }
  NativeMissionCommandGateway &gateway_;
  std::optional<Draft> draft_{};
};

}  // namespace dyx::rpp
