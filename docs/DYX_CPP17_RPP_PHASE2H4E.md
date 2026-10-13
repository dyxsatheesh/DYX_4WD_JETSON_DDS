# DYX Autonomous — C++17 Phase 2H4E

**Mission Upload, Edit/Update, Preview and Explicit LOAD Contract**

**Prepared by: Satheesh**
**Baseline:** `Jetson_DDS_Upgrade` commit `772fa08` (Phase 2H4D).
**Verification strategy:** offline Visual Studio 2022 C++17 tests; Jetson/PX4/ROS 2 verification deferred.

## Purpose

Implement the intended client workflow without connecting to any hardware:

1. **UPLOAD** marking positions; compile a trajectory and a complete SHA-256-consistent prepared-path snapshot. Return its signature and revision for map preview. Upload **does not load** or start the mission.
2. **Review/edit** the preview in the client. The old saved draft stays unchanged until the user explicitly chooses **UPDATE**.
3. **UPDATE** with the prior revision and signature; validate the complete new trajectory and atomically replace the saved draft. A failed edit leaves the previous preview intact.
4. **LOAD** using the confirmed revision and signature; independently rebuild the `NativeMissionInstallation` binding and forward its verified marking identities into `NativeMissionCommandGateway::load()`. The gateway enters **READY**, never START or physical motion. A load request must not contain new marking coordinates.

Backend routes in `backend_mission_upload.hpp`: `POST /api/v1/mission/upload`, `POST /api/v1/mission/update`, `GET /api/v1/mission/draft`, and `POST /api/v1/mission/load`.

## Contracts and safeguards

- Every mutation requires a server-verified authenticated principal, control lease, valid unique run ID, operator session ID, positive per-draft command sequence and trusted monotonic host time.
- UPLOAD, UPDATE and LOAD require a **trusted independently confirmed PX4 disarmed state**. Caller-provided bools and C++ test fixtures are NOT authentication or real PX4 evidence.
- UPDATE and LOAD require an exact current draft **revision AND SHA-256 signature**. Stale or reordered commands are rejected. Retrying a failed command requires a new sequence.
- Draft ownership is bound to the upload's operator session. A foreign session is rejected. Denied/error responses do not expose draft coordinates, signature or counts.
- The generated path uses local **ENU x=east, y=north, metres**. Raw WGS84 coordinates must be transformed and validated by a future ingress adapter; do **not** pass degrees directly.
- The extension toggle is initially OFF; default extension length 4.0 m, editable within **0.10–20.0 m**; default trigger strictly **<3.0 m**. The existing trajectory builder only creates row-end extension points for eligible transitions. Two-point missions have no extension dummy.
- Duplicate run IDs are rejected by the native gateway even after STOP; a new run ID is required for a new LOAD.
- Discard of an **unloaded** draft requires independent operator and disarm authorizations; a loaded mission cannot be discarded as an alternative to STOP.
- Verified marking goal identities are derived from the run ID, snapshot signature and raw navigation index; the preview includes non-actuating counts and SHA-256 digest, not an authorization token.
- The module is designed for serialized calls from a single trusted event loop. Thread-safe synchronization, persistent storage, JSON validation, authentication, authorization, TLS, API rate limits, and network server remain separate pending integration tasks.

## Offline tests

Adds four Windows-native CTest tests:

- `backend_upload_nominal_native_smoke` — UPLOAD → GET PREVIEW → LOAD READY only, identities, exact generated path.
- `backend_upload_update_native_smoke` — explicit UPDATE, real serpentine extension, revision conflict, failed edit atomicity, unsaved LOAD rejection.
- `backend_upload_authority_native_smoke` — authentication, lease, disarm, ownership, replay, time, extension validation, sensitive response redaction, authorized draft discard.
- `backend_upload_load_safety_native_smoke` — disarm denial, STOP, run ID replay prevention, newly authorized run.

The expected total is **71 native C++17 tests** (67 existing + 4 new). GCC local verification and the Windows MSVC check must be recorded separately.

## Not implemented or claimed

**NO network endpoint, JSON transport, Android authentication, TLS, live PX4 DDS, actual physical STOP, spray, laser or wheel motion.** No hardware timing or GNSS positioning accuracy claim is made. The installed mission is a *decision-only* READY state. The Android application and production C++17 server integration, reliable persistent journal, and Jetson/PX4 hardware acceptance remain to be done.
