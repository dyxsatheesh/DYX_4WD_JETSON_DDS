#pragma once
// Phase 2G2C: NON-ACTUATING installation of a verified prepared mission
// into the native C++17 mission sequencer. This file neither starts a mission
// nor publishes any ROS/DDS/PX4 command. Python and MAVROS are not required.
//
// SHA-256 is a SNAPSHOT CONSISTENCY check, not an authentication mechanism.
// A trusted caller must authorize upload, provide a unique run ID, and verify
// the vehicle is disarmed before invoking install_from_pending().

#include "dyx_rpp_cpp17_core/mission_snapshot.hpp"
#include "dyx_rpp_cpp17_core/mission_sequence.hpp"
#include <cstddef>
#include <memory>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dyx::rpp {

inline bool valid_native_mission_run_id(const std::string &id) {
  if (id.size() < 8 || id.size() > 128) return false;
  for (unsigned char c : id) {
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '-' || c == '_')) return false;
  }
  return true;
}

inline std::string native_goal_instance_id(const std::string &run_id,
                                           const std::string &signature,
                                           int raw_index,
                                           std::size_t semantic_sequence) {
  if (!valid_native_mission_run_id(run_id) || !valid_path_signature(signature) ||
      raw_index < 0) throw std::invalid_argument("invalid goal identity input");
  // Byte-for-byte contract: mission_manager/path_contract.py:
  // sha256(f"{run}|{path}|{raw}|{semantic_goal_sequence}")
  const std::string material = run_id + "|" + signature + "|" +
      std::to_string(raw_index) + "|" + std::to_string(semantic_sequence);
  snapshot_detail::Sha256 sha;
  sha.text(material.c_str());
  return sha.hex();
}

struct NativeMissionInstallation {
  PreparedPathSnapshot snapshot;
  PathGeometryIndex geometry;
  std::string mission_run_id;
  std::vector<MarkingGoal> marking_goals;
  MissionSequencer sequencer;

  NativeMissionInstallation(PreparedPathSnapshot verified,
                            std::string run_id, MissionMode mode)
      : snapshot(std::move(verified)),
        geometry(PathGeometryIndex::build(snapshot.raw_navigation())),
        mission_run_id(std::move(run_id)) {
    if (!valid_native_mission_run_id(mission_run_id) ||
        !valid_path_signature(snapshot.signature) ||
        compute_path_signature(snapshot) != snapshot.signature)
      throw std::invalid_argument("unverified mission snapshot");

    std::size_t semantic_sequence = 0;
    for (std::size_t i = 0; i < snapshot.point_types.size(); ++i) {
      const int type = snapshot.point_types[i];
      if (type != point_marking && type != point_dummy_alignment) continue;
      if (type == point_marking) {
        const int index = snapshot.marking_indices[i];
        if (index < 0 || static_cast<std::size_t>(index) != marking_goals.size())
          throw std::invalid_argument("marking index out of order");
        if (i > static_cast<std::size_t>(std::numeric_limits<int>::max()))
          throw std::length_error("raw path index overflow");
        const std::string label = "P" + [] (int n) {
          std::string digits = std::to_string(n);
          if (digits.size() < 4) digits.insert(0, 4 - digits.size(), '0');
          return digits;
        }(index + 1);
        const std::string instance = native_goal_instance_id(
            mission_run_id, snapshot.signature, static_cast<int>(i), semantic_sequence);
        const std::string identity = "RUN:" + mission_run_id +
          "|PATH:" + snapshot.signature + "|RAW:" + std::to_string(i) +
          "|GOAL:" + label + "|INSTANCE:" + instance;
        marking_goals.push_back({static_cast<int>(i), index, label, identity});
      }
      ++semantic_sequence;
    }
    if (marking_goals.size() != snapshot.marking_points.size())
      throw std::invalid_argument("marking goals incomplete");
    sequencer.load(marking_goals, mode); // READY ONLY, not START
  }
};

enum class NativeInstallStatus {
  installed, already_active, unauthorized, vehicle_not_disarmed,
  invalid_run_id, run_id_reused, snapshot_not_ready, snapshot_invalid,
  binding_failed, not_active, not_stopped
};
inline const char *native_install_status_name(NativeInstallStatus s) {
  switch (s) {
    case NativeInstallStatus::installed: return "INSTALLED";
    case NativeInstallStatus::already_active: return "ALREADY_ACTIVE";
    case NativeInstallStatus::unauthorized: return "UNAUTHORIZED";
    case NativeInstallStatus::vehicle_not_disarmed: return "VEHICLE_NOT_DISARMED";
    case NativeInstallStatus::invalid_run_id: return "INVALID_RUN_ID";
    case NativeInstallStatus::run_id_reused: return "RUN_ID_REUSED";
    case NativeInstallStatus::snapshot_not_ready: return "SNAPSHOT_NOT_READY";
    case NativeInstallStatus::snapshot_invalid: return "SNAPSHOT_INVALID";
    case NativeInstallStatus::binding_failed: return "BINDING_FAILED";
    case NativeInstallStatus::not_active: return "NOT_ACTIVE";
    case NativeInstallStatus::not_stopped: return "NOT_STOPPED";
  }
  return "INVALID";
}
struct NativeInstallResult {
  NativeInstallStatus status{NativeInstallStatus::snapshot_not_ready};
  std::string detail{};
  bool installed() const { return status == NativeInstallStatus::installed; }
};

class NativeMissionInstaller final {
 public:
  NativeInstallResult install_from_pending(PendingPreparedSnapshot &pending,
                                          const std::string &mission_run_id,
                                          MissionMode mode,
                                          bool authenticated_upload_authorized,
                                          bool px4_disarmed_confirmed) {
    if (installed_) return {NativeInstallStatus::already_active, "EXPLICIT_STOP_AND_RETIRE_REQUIRED"};
    if (!authenticated_upload_authorized) return {NativeInstallStatus::unauthorized, "UPLOAD_AUTHORIZATION_REQUIRED"};
    if (!px4_disarmed_confirmed) return {NativeInstallStatus::vehicle_not_disarmed, "DISARM_REQUIRED"};
    if (!valid_native_mission_run_id(mission_run_id)) return {NativeInstallStatus::invalid_run_id, "INVALID_MISSION_RUN_ID"};
    if (used_run_ids_.count(mission_run_id)) return {NativeInstallStatus::run_id_reused, "RUN_ID_REPLAY"};

    const auto attempt = pending.try_install();
    if (!attempt.committed()) {
      const auto status = attempt.status == SnapshotInstallStatus::not_ready ||
          attempt.status == SnapshotInstallStatus::incomplete
          ? NativeInstallStatus::snapshot_not_ready : NativeInstallStatus::snapshot_invalid;
      return {status, snapshot_install_status_name(attempt.status)};
    }
    try {
      // Build all C++ objects before publishing an installed pointer.
      auto candidate = std::make_unique<NativeMissionInstallation>(
          *attempt.snapshot, mission_run_id, mode);
      used_run_ids_.insert(mission_run_id);
      installed_ = std::move(candidate);
    } catch (const std::exception &e) {
      // Fail closed. Staging consumed this signature; the caller must explicitly
      // reset pending sources at a safe mission boundary before another install.
      return {NativeInstallStatus::binding_failed, e.what()};
    }
    return {NativeInstallStatus::installed, "READY_NON_ACTUATING"};
  }

  // Read-only access deliberately does not provide START, NEXT, motor, or
  // spray authority. Those belong in a future safety-arbitrated runtime.
  const NativeMissionInstallation *current() const { return installed_.get(); }

  // A transport-independent STOP request for the sequencer state only.
  // Production code must independently send/confirm the PX4 safe-stop.
  NativeInstallResult stop_sequence(bool authenticated_operator_authorized) {
    if (!installed_) return {NativeInstallStatus::not_active, "NO_INSTALLED_MISSION"};
    if (!authenticated_operator_authorized) return {NativeInstallStatus::unauthorized, "OPERATOR_AUTHORIZATION_REQUIRED"};
    installed_->sequencer.stop();
    return {NativeInstallStatus::installed, "SEQUENCE_STOPPED_NOT_PX4_STOP"};
  }

  NativeInstallResult retire(bool authenticated_operator_authorized,
                             bool px4_disarmed_confirmed) {
    if (!installed_) return {NativeInstallStatus::not_active, "NO_INSTALLED_MISSION"};
    if (!authenticated_operator_authorized) return {NativeInstallStatus::unauthorized, "OPERATOR_AUTHORIZATION_REQUIRED"};
    if (!px4_disarmed_confirmed) return {NativeInstallStatus::vehicle_not_disarmed, "DISARM_REQUIRED"};
    const auto stage = installed_->sequencer.stage();
    if (stage != MissionStage::stopped && stage != MissionStage::finished)
      return {NativeInstallStatus::not_stopped, "STOP_MISSION_BEFORE_RETIRE"};
    installed_.reset();
    // used_run_ids_ intentionally persists for this process lifetime.
    return {NativeInstallStatus::installed, "RETIRED_NON_ACTUATING"};
  }
 private:
  std::unique_ptr<NativeMissionInstallation> installed_{};
  std::set<std::string> used_run_ids_{};
};
} // namespace dyx::rpp
