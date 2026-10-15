# Phase 2H4O — Android Mission Upload, Update & LOAD Compatibility (OFFLINE)

**DYX Autonomous · C++17 · Prepared by Satheesh**

## Purpose

Provide an offline, transport-free native C++17 Android UI *proposal* model for **UPLOAD → optional edit/UPDATE → explicit LOAD**. This complements the existing backend `NativeBackendMissionUpload` validation and the Phase 2H4N Android command proposal contract. The actual Android app, HTTP listener mutation endpoints, Jetson DDS bridge, PX4 and rover hardware are NOT connected by this phase.

## Rules

- Android supplies local ENU marking points (metres), extension ON/OFF + range 0.10–20.0 m and mission mode. Client checks basic bounds; the **trusted backend alone** computes and validates the complete navigation path, immutable snapshot signature and marking indices. These are NOT WGS84 coordinates.
- Initial UPLOAD stages preview. UPDATE includes the last *fresh* backend revision + signature, rejects stale edits and obtains a new revision. LOAD sends **no geometry**: only the exact backend revision/signature approved by the operator. The backend must independently enforce PX4 DISARMED, authorization and lease.
- Explicit operator LOAD confirmation is required and unsaved local edits prevent LOAD. Client must display preview and resync after every response; a lost response is **UNKNOWN**, not safe to retry with the same sequence.
- The UI's sign-in, lease and disarmed indicators are **not security attestations**. A privileged backend adapter independently supplies `BackendVerifiedPrincipal` and `trusted_px4_disarmed`; neither is present in an Android proposal.
- Mission-run changes, sign-out, lease changes, disconnects, request timeouts, stale previews and monotonic-clock rollback fence UI proposals. Sequence never resets during an instance's lifetime. No Android screen state can authorize a rover start.
- LOAD success means **mission prepared / READY, not started**. A signature/digest is consistency checking, **not cryptographic origin authentication**. This model never emits network, servo, laser or motor commands.

## Native regression tests

Four tests bring the full offline suite to **112**:

1. `backend_android_editor_nominal_native_smoke` — complete typed UPLOAD → UPDATE → LOAD against the actual native backend and mission gateway, verifies READY and no actuation.
2. `backend_android_editor_safety_native_smoke` — no lease, disarm hint, invalid geometry, unconfirmed/stale LOAD, forged hardware-output reply, duplicate pending command.
3. `backend_android_editor_timeout_native_smoke` — uncertain result, no automatic retry, monotonic sequence, host clock fault and independently verified DISARMED recovery.
4. `backend_android_editor_epoch_native_smoke` — mission identity fencing, replay reply, disconnect, invalid preview signature, sign-out.

### Windows offline commands

`cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64`

`cmake --build build/dyx_cpp_only --config Release`

`ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure`

Do not claim Windows pass until user tests. Build warnings on `/DNDEBUG` and `/UNDEBUG` reflect assertion-enabled native tests and are non-fatal.

## Pending before production

Create real strict JSON schema, authenticated TLS client and backend route implementation, GET /draft replay-safe synchronization with mission identity, physical PX4 disarm validation, launch-ready Jetson/ROS2 DDS integration, Android runtime UI tests and end-to-end no-actuation simulations. The existing loopback demo remains READ-ONLY. No motion/marking proof is established by these offline tests.
