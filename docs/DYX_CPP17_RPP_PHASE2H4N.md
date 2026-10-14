# Phase 2H4N — Android Mission Control Compatibility (OFFLINE)

DYX Autonomous 4WD marking rover · Native C++17 · Prepared for team verification.

## Purpose

Adds `backend_android_mission_controls.hpp`: an offline C++17 model of mission-command proposals produced by a future Android client. It complements Phase 2H4M's **read-only telemetry replica** and verifies UI state-machine behavior for START, PAUSE, RESUME, NEXT, SKIP and STOP. It does **not** build an Android application or authorize a rover command.

## Behavior tested

- Proposals use fixed backend routes and a strict two-field request body (`mission_run_id` and positive monotonic `sequence`); no authentication, lease or physical-command claims are encoded in client JSON.
- START requires a current backend mission snapshot in READY, no unsaved mission edits, displayed control lease and fresh valid RTK FIXED. RESUME requires PAUSED. NEXT/SKIP require an applicable resolved/await-next state; SKIP requires explicit operator confirmation.
- PAUSE does not require RTK FIXED; a running mission may still need a pause during RTK loss. The authoritative backend safety gateway makes the final decision.
- STOP can be **proposed** despite stale RTK, safety hold, missing control lease, host clock fault or another pending command. It still needs an authenticated backend path and a valid run identity; an offline/unreachable backend cannot be claimed stopped. Physical E-stop and PX4 failsafes MUST remain independent of the Android UI and HTTP.
- Only one ordinary request can be pending; responses are sequence-correlated; timeout/unknown result never auto-retries an already used sequence. Replayed or old-mission decisions are rejected.
- Server response `MISSION_DECISION_ACCEPTED` means **decision accepted**, never that a motor/spray/laser actually executed. All fixtures are non-actuating. A malformed/unsafe response cannot claim actuation.
- Monotonic clock rollback is latched for non-STOP commands. Recovering the demo model requires independently verified operator approval and PX4 DISARMED. Sequence counters never reset within an instance.

## Boundaries

This is an **offline UI compatibility proposal model**. All typed backend replies and telemetry views come from trusted in-process test fixtures; a real Android Kotlin/Compose or React Native client must perform strict JSON decoding, TLS and server-certificate validation, authentication, request/response correlation, replay-resistant session handling, UI reconnection, and command-delivery/error display. Real backend control leases, RTK freshness, permissions, motion and STOP must be enforced independently of all Android flags. No web server becomes writable in this patch: the loopback server remains **read-only**. Upload/update/load remain separate native backend contracts; they are not wired to a real Android upload UI here. This phase does not test Jetson, ROS2, DDS, PX4 or hardware.

## Native regression tests

Four new tests; expected full total **108**:

1. `backend_android_controls_nominal_native_smoke`: strict route/body, START/PAUSE/RESUME, ACK correlation and one-request-inflight.
2. `backend_android_controls_safety_native_smoke`: STOP despite unsafe RTK and missing lease, offline stop delivery uncertainty, SKIP confirmation, unsaved edits and hostile reply.
3. `backend_android_controls_timeout_native_smoke`: no automatic retry, expired request result unknown and new monotonic sequence.
4. `backend_android_controls_epoch_native_smoke`: run/session fencing, host clock rollback and independently authorized disarmed recovery.

On Windows, configure `src/dyx_rpp_cpp17_core/test/native_cpp_only` with Visual Studio 17 2022 x64, build Release, run `ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure`. Commit only if **108/108** pass. Normal `/DNDEBUG` overridden by `/UNDEBUG` warnings on assertion-based native tests are non-fatal.
