#include "backend_http_fixture.hpp"
#include "backend_upload_http_fixture.hpp"
#include "backend_android_controls_fixture.hpp"
#include <cassert>
#include <string>

using namespace dyx::rpp;

static void status() {
  NativeMissionCommandGateway g;
  NativeBackendMissionApi api(g);
  NativeBackendLiveStatus live(g);
  NativeBackendHttpWire wire(api, live);
  const auto original = wire.handle(http_read(kBackendMissionStatusPath, 1.0));
  assert(original.status == 200);
  assert(original.body.find("\"recovery_quarantined\":false") != std::string::npos);
  g.latch_recovery_quarantine();
  const auto quarantined = wire.handle(http_read(kBackendMissionStatusPath, 1.1));
  assert(quarantined.status == 200);
  assert(quarantined.body.find("\"recovery_quarantined\":true") != std::string::npos);
  assert(quarantined.body.find("\"hardware_command_emitted\":false") != std::string::npos);
}

static void commands() {
  NativeMissionCommandGateway g;
  g.latch_recovery_quarantine();
  NativeBackendMissionApi api(g);
  NativeBackendLiveStatus live(g);
  NativeBackendHttpWire wire(api, live);
  for (const char* path : {kBackendMissionStartPath, kBackendMissionPausePath,
       kBackendMissionResumePath, kBackendMissionNextPath, kBackendMissionSkipPath}) {
    const auto r = wire.handle(http_post(path, 1, 1.0));
    assert(r.status == 423);
    assert(r.body.find("MISSION_RECOVERY_QUARANTINED") != std::string::npos);
    assert(r.http11_bytes().find("HTTP/1.1 423 Locked\r\n") == 0);
    assert(!r.physical_command_emitted && !r.network_io_performed);
  }
  assert(!g.has_active_mission() && g.recovery_quarantined());
}

static void stop() {
  NativeMissionCommandGateway g;
  g.latch_recovery_quarantine();
  NativeBackendMissionApi api(g);
  NativeBackendLiveStatus live(g);
  NativeBackendHttpWire wire(api, live);
  auto req = http_post(kBackendMissionStopPath, 1, -1.0);
  req.principal.owns_control_lease = false;
  const auto r = wire.handle(req);
  assert(r.status == 200);
  assert(r.body.find("\"safe_stop_intent\":true") != std::string::npos);
  assert(g.recovery_quarantined() && g.safe_stop_requested());
  assert(!g.emits_hardware_commands());
}

static void upload() {
  NativeMissionCommandGateway g;
  NativeBackendMissionUpload backend(g);
  NativeBackendUploadHttpWire wire(backend);
  g.latch_recovery_quarantine();
  const auto q = draft_http("POST", kBackendMissionUploadPath, draft_upload_json());
  const auto r = wire.handle(q);
  assert(r.status == 423);
  assert(r.body.find("RECOVERY_QUARANTINE_COMMANDS_LOCKED") != std::string::npos);
  assert(r.http11_bytes().find("HTTP/1.1 423 Locked\r\n") == 0);
  assert(!backend.has_draft() && !g.has_active_mission());
  // The ordinary draft GET route is also locked; the H5K separately
  // authorized recovery PREVIEW reader must be used instead.
  const auto read = wire.handle(draft_get());
  assert(read.status == 423);
}

static void android_status() {
  NativeOfflineAndroidMissionControls screen;
  auto ctx = android_controls_fixture();
  ctx.mission.recovery_quarantined = true;
  screen.show(ctx);
  assert(screen.recovery_quarantine_latched());
  assert(screen.propose(AndroidMissionIntent::start, 100.1).code ==
         "ANDROID_RECOVERY_QUARANTINED");
  const auto stop_request = screen.propose(AndroidMissionIntent::stop, 100.2);
  assert(stop_request.prepared && stop_request.path == kBackendMissionStopPath);
  android_no_actuation(stop_request);
  ctx.mission.recovery_quarantined = false; // stale/replayed healthy status
  screen.show(ctx);
  assert(screen.propose(AndroidMissionIntent::start, 100.3).code ==
         "ANDROID_RECOVERY_QUARANTINED");
}

static void android_reply() {
  NativeOfflineAndroidMissionControls screen;
  auto ctx = android_controls_fixture();
  screen.show(ctx);
  const auto request = screen.propose(AndroidMissionIntent::start, 100.1);
  assert(request.prepared);
  BackendMissionApiResponse denied;
  denied.http_status = 423;
  denied.code = "MISSION_RECOVERY_QUARANTINED";
  denied.recovery_quarantined = true;
  const auto ack = screen.observe_backend(request.sequence, denied, 100.2);
  assert(ack.code == "ANDROID_RECOVERY_QUARANTINED");
  assert(ack.backend_decision_reported && !ack.executed_on_rover);
  assert(!screen.request_pending() && screen.recovery_quarantine_latched());
  screen.show(ctx);
  assert(screen.propose(AndroidMissionIntent::start, 100.3).code ==
         "ANDROID_RECOVERY_QUARANTINED");
  assert(screen.propose(AndroidMissionIntent::stop, 100.4).prepared);
}

static void authentication() {
  NativeMissionCommandGateway g;
  g.latch_recovery_quarantine();
  NativeBackendMissionApi api(g);
  NativeBackendLiveStatus live(g);
  NativeBackendHttpWire wire(api, live);
  auto read = http_read(kBackendMissionStatusPath, 1.0);
  read.principal.authenticated = false;
  assert(wire.handle(read).status == 401);
  auto command = http_post(kBackendMissionStartPath, 1, 1.0);
  command.principal.authenticated = false;
  assert(wire.handle(command).status == 401);
  assert(!g.has_active_mission());
}

int main(int argc, char** argv) {
  assert(argc == 2);
  const std::string mode(argv[1]);
  if (mode == "status") status();
  else if (mode == "commands") commands();
  else if (mode == "stop") stop();
  else if (mode == "upload") upload();
  else if (mode == "android_status") android_status();
  else if (mode == "android_reply") android_reply();
  else if (mode == "authentication") authentication();
  else return 2;
  return 0;
}
