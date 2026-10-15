# Phase 2H4P — Native C++17 Mission Draft HTTP/JSON Contract (OFFLINE)

**DYX Autonomous · Prepared by Satheesh · 15 October 2026**

## Scope

Adds the native, transport-neutral `NativeBackendUploadHttpWire` service over the pre-existing `NativeBackendMissionUpload` and `NativeMissionCommandGateway`. This is **not connected to a listening socket**: the current `backend_loopback_readonly_demo` remains READ-ONLY, with POST rejected. No Jetson, ROS2, DDS, PX4, motor, laser or spray is invoked.

The new service uses explicitly **trusted** request metadata: independently authenticated `BackendVerifiedPrincipal`, monotonic server receive time and fresh PX4 disarm attestation from a privileged adapter. None are read from JSON. Android screen hints do not confer privilege.

## Intended JSON routes / request contract

**UPLOAD** — `POST /api/v1/mission/upload` (initial preview, no movement)

```json
{"mission_run_id":"RUN_H4P_0001","sequence":1,"marking_points":[[0,0],[2,0]],"extension_enabled":false,"extension_distance_m":4.0,"mission_mode":"automatic"}
```

**UPDATE** — `POST /api/v1/mission/update` (revision/signature and complete replacement geometry required)

```json
{"mission_run_id":"RUN_H4P_0001","sequence":2,"expected_revision":1,"expected_signature":"<64 lowercase hex characters from GET /draft>","marking_points":[[0,0],[3,0]],"extension_enabled":false,"extension_distance_m":4.0,"mission_mode":"automatic"}
```

**LOAD** — `POST /api/v1/mission/load` (revision/signature only, NO geometry)

```json
{"mission_run_id":"RUN_H4P_0001","sequence":3,"expected_revision":2,"expected_signature":"<64 lowercase hex characters from GET /draft>"}
```

**GET DRAFT** — `GET /api/v1/mission/draft?mission_run_id=RUN_H4P_0001` with empty body and authenticated mission-read scope, matching owner session. No other query parameters are accepted. LOAD returns `MISSION_LOADED_READY_NOT_STARTED`; the mission remains in READY, not STARTED. All coordinates are **local ENU metres**, never latitude/longitude.

A successful decision response supplies `draft.revision`, `draft.signature`, counts and loaded/preview flags; it does **not** claim physical execution. A signature/digest verifies consistency, **not cryptographic sender identity**. A rejected result contains no other operator's draft signature.

## Validation and safety

- Only the exact fields appropriate to the requested route are accepted; field order may vary; duplicates, unknown keys, trust flags, non-ASCII/escaped token identities, coercions, trailing junk, malformed points, integer overflow and nonfinite floats are rejected.
- The JSON numeric grammar is strict, decimal point formatting is locale-independent, and point/body/response sizes are bounded (default body 256 KiB, up to 10,000 points).
- Backend—not parser—rechecks geometry, immutable snapshot signature, revision fencing, ownership, sequence, control lease, PX4 disarm and mission stage. Unauthorized access fails closed.
- `backend_http_wire.hpp` gains status 422 serialization for invalid mission geometry. This is a legitimate HTTP error response, not a success.
- No successful UPLOAD/UPDATE/LOAD can bypass the previously implemented backend safety gate. Client request flags cannot set authentication/lease/PX4 state. No physical commands are emitted by this phase.

## Offline regression

Four new native C++17 tests: `backend_upload_http_nominal_native_smoke`, `backend_upload_http_security_native_smoke`, `backend_upload_http_parser_native_smoke`, and `backend_upload_http_safety_native_smoke`.

All **116/116** native tests pass in local GCC Release build. The patch was also checked against exact upstream Git blob identities for both modified files. Windows Visual Studio 2022 Release build/CTest remains for team verification, expected 116/116.

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

## Not yet connected / required for production

- HTTP listener still **READ-ONLY**; this is a validated in-memory request/response contract, **not** a writable real HTTP server.
- A separate authenticated TLS ingress must reject unsupported content types, HTTP request smuggling, CSRF/cross-origin misuse, spoofed headers and identity/lease/PX4 fields before producing a trusted request.
- Jetson ROS2 / XRCE-DDS transport and PX4 disarm state must be independently integrated and hardware tested.
- Real Android app request/response parsing, coherent GET /draft resync, end-to-end replay/timeout tests, actuator watchdog, RTK/battery interlocks and field tracking remain pending.

**Production deployment and rover actuation are NOT approved from these offline results.**
