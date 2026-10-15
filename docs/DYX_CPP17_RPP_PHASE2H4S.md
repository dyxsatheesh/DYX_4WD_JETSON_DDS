# DYX Autonomous — Phase 2H4S: Offline Android Draft Transaction Coordinator

**Baseline:** `Jetson_DDS_Upgrade` commit `c0cf4be` (Phase 2H4R).
**Scope:** C++17 offline contract and deterministic Windows/Linux native tests only.

## What changed

A single `NativeOfflineAndroidDraftCoordinator` composes the previously tested
mission editor, strict POST HTTP/JSON codec, and authoritative GET /draft sync.
The coordinator prepares requests; it does not send them.

- Keeps a **single in-flight request**, and blocks mutations while a GET is
  pending (and vice versa).
- **Strips untrusted cached revision/signature from Android UI screen inputs.**
  Only a verified GET /draft can refresh backend state.
- Requires an externally authenticated, path/sequence/body-correlated response
  and bounded host monotonic time before accepting a backend decision.
- Any unknown/malformed/late/tampered mutation response consumes its request,
  requires a fresh GET, and **never automatically retransmits POST**.
- Successful POST acknowledges only a backend decision, never physical motion;
  a fresh verified GET is required before another mutation.
- Switching mission, signing out, disconnecting, or changing visible lease
  invalidates in-flight requests and draft cache.
- Clock regression latches a coordinator fault (recreate only after independent
  re-authentication and disarm verification). Sequences do not reset mid-session.

## Offline native regression additions

1. `backend_android_coordinator_nominal_native_smoke`: GET empty, UPLOAD, GET,
   UPDATE, GET, LOAD, GET; loaded mission remains READY, with no actuator output.
2. `backend_android_coordinator_security_native_smoke`: untrusted UI revision,
   forged GET transport proof, tampered POST body, stale duplicate reply.
3. `backend_android_coordinator_timeout_native_smoke`: lost POST reply, timeout,
   no auto-retry, authoritative GET recovery.
4. `backend_android_coordinator_epoch_native_smoke`: mission switch fences old
   reply, foreign-owner GET denied, sign-out blocks requests.

**Expected total:** 128 offline CTest cases, contingent on the platform build.

## Security and deployment boundary

No TCP listener or Android client is provided. Trusted metadata is supplied by
an **external privileged transport adapter**, not extracted from JSON or UI.
A boolean `verified_authenticated_transport` is **not authentication**. No TLS,
identity verification, API permission gateway, DDS transport, PX4, RoboClaw,
motor, spray, or laser action is enabled by this phase. Hardware tests remain
mandatory before any physical use.
