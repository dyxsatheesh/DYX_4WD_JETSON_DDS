#include "dyx_rpp_cpp17_core/native_control_owner.hpp"

#include <cassert>
#include <iostream>

using namespace dyx::rpp;

static BackendMissionUploadRequest upload() {
  BackendMissionUploadRequest r{};
  r.path = kBackendMissionUploadPath;
  r.method = "POST";
  r.mission_run_id = "backend_run_001";
  r.sequence = 1;
  r.host_received_sec = 1.0;
  r.principal.authenticated = true;
  r.principal.owns_control_lease = true;
  r.principal.operator_session_id = "backend_session_001";
  r.trusted_px4_disarmed = true;
  r.marking_points = {{0.0, 0.0}, {4.0, 0.0}};
  return r;
}

int main() {
  NativeControlOwner owner;

  auto req = upload();

  // Authentication denial cannot create a draft.
  req.principal.authenticated = false;
  assert(owner.handle_backend_upload(req).http_status == 401);
  assert(owner.prepared_mission() == nullptr);

  req.principal.authenticated = true;
  const auto draft = owner.handle_backend_upload(req);

  assert(draft.http_status == 200);
  assert(draft.draft_ready);
  assert(!draft.loaded);
  assert(owner.prepared_mission() == nullptr);

  // LOAD must reference the saved, immutable preview.
  req.path = kBackendMissionLoadPath;
  req.sequence = 2;
  req.host_received_sec = 2.0;
  req.marking_points.clear();
  req.expected_revision = draft.revision;
  req.expected_signature = draft.signature;

  const auto loaded = owner.handle_backend_upload(req);

  assert(loaded.http_status == 200);
  assert(loaded.loaded);
  assert(owner.prepared_mission() != nullptr);
  assert(owner.has_active_mission());
  assert(owner.mission().stage() == MissionStage::ready);

  // Mission LOAD is not permission to START or drive.
  assert(!owner.hardware_command_emitted());

  BackendMissionApiRequest status{};
  status.method = "GET";
  status.path = kBackendMissionStatusPath;
  status.principal.authenticated = true;
  status.principal.can_read_mission = true;

  const auto snapshot = owner.handle_backend_action(status);
  assert(snapshot.http_status == 200);
  assert(snapshot.stage == MissionStage::ready);

  // Without a fresh real PX4/safety tick, START must fail.
  BackendMissionApiRequest start{};
  start.method = "POST";
  start.path = kBackendMissionStartPath;
  start.mission_run_id = "backend_run_001";
  start.sequence = 3;
  start.host_received_sec = 3.0;
  start.principal.authenticated = true;
  start.principal.owns_control_lease = true;
  start.principal.operator_session_id = "backend_session_001";

  assert(owner.handle_backend_action(start).http_status != 200);
  assert(owner.mission().stage() == MissionStage::ready);

  owner.force_stop();
  assert(!owner.has_active_mission());
  assert(!owner.hardware_command_emitted());

  std::cout
      << "PASS: Backend upload/load/action single-owner integration\n";
}
