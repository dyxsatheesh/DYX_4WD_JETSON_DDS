#pragma once
// Offline C++17 tests; only synthetic local-ENU coordinates, no rover outputs.
#include "dyx_rpp_cpp17_core/backend_mission_upload.hpp"
#include <cassert>
#include <string>
#include <vector>
using namespace dyx::rpp;

inline BackendMissionUploadRequest upload_req(const char *path, std::uint64_t seq,
                                               double time_sec) {
  BackendMissionUploadRequest req;
  req.path = path;
  req.method = "POST";
  req.principal = {true, true, true, "SESSION_H4E_OPERATOR"};
  req.mission_run_id = "RUN_H4E_0001";
  req.sequence = seq;
  req.host_received_sec = time_sec;
  req.trusted_px4_disarmed = true;
  req.marking_points = {{0,0},{2,0}};
  return req;
}
inline BackendMissionUploadRequest load_req(const BackendMissionUploadResponse &draft,
                                             std::uint64_t seq, double time_sec) {
  auto req = upload_req(kBackendMissionLoadPath, seq, time_sec);
  req.marking_points.clear();
  req.expected_revision = draft.revision;
  req.expected_signature = draft.signature;
  return req;
}
inline BackendMissionUploadRequest update_req(const BackendMissionUploadResponse &draft,
                                               std::uint64_t seq, double time_sec) {
  auto req = upload_req(kBackendMissionUpdatePath, seq, time_sec);
  req.expected_revision = draft.revision;
  req.expected_signature = draft.signature;
  return req;
}
inline BackendMissionUploadRequest draft_req() {
  auto req = upload_req(kBackendMissionDraftPath, 0, 0);
  req.method = "GET";
  req.principal.owns_control_lease = false;
  req.marking_points.clear();
  return req;
}
