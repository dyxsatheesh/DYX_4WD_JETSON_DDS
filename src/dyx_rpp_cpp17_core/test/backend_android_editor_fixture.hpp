#pragma once
#include "dyx_rpp_cpp17_core/backend_android_mission_editor.hpp"
#include <cassert>
#include <limits>
using namespace dyx::rpp;
inline AndroidEditorScreenContext editor_screen() {
  AndroidEditorScreenContext s;
  s.mission_run_id="RUN_H4O_0001";
  s.ui_signed_in=true;
  s.ui_control_lease_visible=true;
  s.ui_network_connected=true;
  s.ui_px4_disarmed_indicator=true;
  s.unsaved_local_changes=true;
  s.marking_points={{0,0},{2,0},{2,3}};
  return s;
}
inline BackendMissionUploadResponse editor_backend_draft(std::uint64_t revision=1) {
  BackendMissionUploadResponse r;
  r.http_status=200;
  r.code="DRAFT_PREVIEW_READY";
  r.revision=revision;
  r.signature="0000000000000000000000000000000000000000000000000000000000000000";
  r.marking_count=3;
  r.navigation_count=101;
  r.draft_ready=true;
  r.preview_only=true;
  return r;
}
inline BackendMissionUploadResponse editor_response(const char* code,
      std::uint64_t rev, bool loaded=false) {
  auto r=editor_backend_draft(rev);
  r.code=code;
  r.loaded=loaded;
  r.preview_only=!loaded;
  return r;
}
inline void editor_non_actuating(const AndroidEditorProposal& p) {
  assert(p.backend_authorization_required && !p.network_io_performed &&
         !p.hardware_command_emitted);
}
inline void editor_non_actuating(const AndroidEditorAcknowledgment& a) {
  assert(!a.network_io_performed && !a.hardware_command_emitted);
}
