#pragma once
#include "dyx_rpp_cpp17_core/backend_android_mission_controls.hpp"
#include <cassert>
#include <string>
using namespace dyx::rpp;
inline AndroidControlScreenContext android_controls_fixture(
    MissionStage stage=MissionStage::ready) {
  AndroidControlScreenContext s;
  s.mission_run_id="run_ph2h4n";
  s.ui_signed_in=true;
  s.ui_control_lease_visible=true;
  s.ui_network_connected=true;
  s.mission_status_received_sec=100.0;
  s.mission.http_status=200;
  s.mission.code="STATUS_SNAPSHOT_READ_ONLY";
  s.mission.stage=stage;
  s.mission.stage_name=mission_stage_name(stage);
  s.mission.mission_loaded=true;
  s.mission.total=3;
  s.mission.pending=3;
  s.live.connected=true;
  s.live.requires_resync=false;
  s.live.snapshot_fresh=true;
  s.live.rtk_fixed=true;
  return s;
}
inline BackendMissionApiResponse android_controls_accepted() {
  BackendMissionApiResponse r;
  r.http_status=200;
  r.code="MISSION_DECISION_ACCEPTED";
  return r;
}
inline void android_no_actuation(const AndroidControlDraft& d) {
  assert(d.requires_backend_authorization);
  assert(!d.network_io_performed && !d.hardware_command_emitted);
}
inline void android_no_actuation(const AndroidControlAcknowledgment& a) {
  assert(!a.executed_on_rover && !a.network_io_performed && !a.hardware_command_emitted);
}
