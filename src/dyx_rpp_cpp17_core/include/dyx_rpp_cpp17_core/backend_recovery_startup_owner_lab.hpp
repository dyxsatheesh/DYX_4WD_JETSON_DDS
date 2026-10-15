#pragma once
// DYX Autonomous Phase 2H5N: isolated, NON-PRODUCTION recovery startup owner.
// Solely owns one backend + gateway pair. The constructor latches quarantine
// BEFORE the object can be used and performs ONE offline H5K admission check.
// No HTTP listener, OS service, physical STOP, PX4 or DDS connection exists.
// All identity/review claims must be independently verified by production ingress.

#include "dyx_rpp_cpp17_core/backend_draft_quarantined_boot_guard_lab.hpp"
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

  // Retiring the owner is irreversible. It does NOT release the durable fence
  // or the process-local gateway quarantine. Restart with a new owner only.
  void retire() { boot_.retire(); }

  bool has_active_mission() const { return gateway_.has_active_mission(); }
  bool has_live_backend_draft() const { return backend_.has_draft(); }
  bool emits_hardware_commands() const { return false; }

 private:
  // Declaration order matters: gateway -> backend -> H5K guard.
  NativeMissionCommandGateway gateway_{};
  NativeBackendMissionUpload backend_;
  NativeOfflineQuarantinedBootGuardLab boot_;
  DraftQuarantinedBootGuardLabResult boot_admission_{};
};
} // namespace dyx::rpp
