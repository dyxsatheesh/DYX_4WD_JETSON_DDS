# Phase 2H4C — Native C++17 client mission command gateway

**DYX Autonomous | Prepared by Satheesh | 13 October 2026**

**Baseline:** Phase 2H4B, `Jetson_DDS_Upgrade` commit `c5156ec`.

## Purpose

Provide a pure C++17, **non-actuating** command boundary for Android/API mission
START, PAUSE, RESUME, NEXT, SKIP and STOP. This layer wraps the existing
`NativeDdsMissionRuntime`, `NativeSafetyMissionBridge` and C++17 sequencer.
It must not expose a ROS publisher, call PX4, change motor outputs, operate
AUX5/AUX6, or manufacture marking success or physical stop acknowledgments.

## Command contract

- `load(...)` takes independently authenticated upload/control-lease/disarmed
  attestations and already verified per-goal identities from the snapshot
  installer. The native wrapper is NOT an authentication system. Never populate
  trusted booleans directly from untrusted Android/client JSON.
- Each mission run and authorized operator session has a validated identifier.
  Mission run IDs cannot be reused within this controller process lifetime;
  production must persist run-id replay protection across restarts.
- Each command contains strictly increasing `session_sequence`, run identity,
  bound operator session, and backend **host monotonic** receipt timestamp.
  Invalid or old identity, repeated/reordered sequence, lease loss, and bad
  time cannot authorize motion. An action that already failed a valid safety
  check cannot be replayed with the same sequence after conditions change.
- A fresh DDS telemetry observation invalidates the underlying motion gate;
  a fresh accepted safety/mission `tick` is required before each motion-authorizing
  action, and each successful action consumes the current tick's authorization.
  The wrapped runtime independently rechecks the telemetry revision and 0.25 s
  safety authority expiry.
- PAUSE requires an authorized operator lease and is independent of the motion
  health status; it does not require a fresh authorization tick. A PAUSE
  requested in a non-pausable stage is rejected, not reported as completed.
- Client STOP requires a bound authenticated session but **not** a control
  lease, healthy RTK, good battery, or a valid receive time. A separate
  `force_stop()` path is unconditional for E-stop/watchdog adapters.
- Both STOP paths stop the **native mission state only** and issue an advisory
  safe-stop intent, including immediately on telemetry epoch-integrity faults.
  PX4 zero command, spray/laser release, motor disable and
  feedback are independent hardware/ROS integration work.
- A detected PX4 reboot/telemetry integrity fault stops the native mission.
  Only independently authorized, disarmed PX4 epoch reset can begin a new run;
  the old run ID remains blocked. No stale mission auto-resume.

## Files and native tests

- `mission_command_gateway.hpp`: safe C++17 operator command gateway.
- `mission_command_gateway_fixture.hpp`: synthetic telemetry test fixture.
- `mission_gateway_nominal_native_smoke.cpp`: START/SKIP/NEXT, one-action-per-tick, STOP.
- `mission_gateway_authority_native_smoke.cpp`: auth, lease, identity, replay, host time.
- `mission_gateway_safety_native_smoke.cpp`: obstacle, stale authorization, safe STOP.
- `mission_gateway_epoch_native_smoke.cpp`: PX4 restart, disarmed reset, run replay.

All four tests run in the ROS-free Windows CMake/CTest lane. Expected result:
**63/63 native C++17 tests**. GCC C++17 testing does not establish Windows
MSVC, ROS 2 Humble, PX4 DDS or physical safety compatibility.

## Production backlog

- C++17 HTTP/WebSocket transport and real identity/token validation with
  permission, one-controller lease, command-rate limiting, audit log and
  persistent sequence/replay database.
- Complete signed snapshot installation binding and run lifecycle persistence.
- Motor zero-output watchdog and independent PX4/actuator safe-stop ACK.
- Restore authorized operation only after telemetry, RC handoff and E-stop
  are physically verified. Jetson and field tests remain deferred.
