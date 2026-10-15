# Phase 2H4Q — Native C++17 Android Mission Draft HTTP Codec (OFFLINE)

**DYX Autonomous · Prepared by Satheesh · 15 October 2026**

## Purpose

Completes one offline, transport-neutral JSON round trip between the Android mission-editor proposal contract from Phase 2H4O and the native mission draft HTTP/JSON server contract from Phase 2H4P. The new `NativeOfflineAndroidDraftHttpCodec` serializes typed proposals to validated UTF-8 ASCII JSON and decodes a narrowly specified canonical success response back to `BackendMissionUploadResponse` for the existing editor's `observe_backend` method.

**The codec is NOT the Android application, a running server, or a secure network client.** No socket, HTTP/TLS exchange, authentication, Jetson, ROS2, DDS, PX4, motor, spray or laser I/O occurs. The current loopback HTTP listener remains **READ-ONLY**. No POST has been opened on a socket.

## Data flow and strict trust separation

1. Existing `NativeOfflineAndroidMissionEditor::propose(...)` produces a non-executing, typed `AndroidEditorProposal` after its own screen-state checks.
2. `NativeOfflineAndroidDraftHttpCodec::encode(proposal)` validates the exact route, mission ID, monotonic sequence, required revision/signature, local ENU point list, point limits, valid motion mode and extension distance, then creates **only** a request payload and method/path. No principal, scope, lease, PX4 disarm flag, token or transport trust is sent as an authority claim.
3. A **future privileged, authenticated transport adapter** must independently establish endpoint identity/TLS/session trust, correlate the response with the submitted request/sequence/path, and supply independent monotonic send/receive timestamps. The `verified_*` fields are supplied by that adapter, not by the server response, request JSON or UI.
4. `decode(verified_reply, original_encoded_request)` rejects unmatched, unauthenticated, stale, negative-time, error-status, oversized and unexpected-content-type responses. It parses only the versioned canonical Phase 2H4P success schema, rejects extra/duplicate/unexpected fields and checks revision/signature/count/ready/loaded/preview consistency.
5. A verified, decoded **decision** can then be passed to the existing editor's `observe_backend(original_sequence, decoded.backend, current_monotonic_time)`; editor timeout/reconnect/resync and authorization rules still apply. A decoded result is **NOT proof of physical execution**.

## Supported POST JSON bodies

- `POST /api/v1/mission/upload` — mission run ID, sequence, marking points (local ENU metres), extension enable/distance, mission mode. Must not carry revision/signature.
- `POST /api/v1/mission/update` — all upload fields plus **expected revision and exact 64-character lowercase path signature**.
- `POST /api/v1/mission/load` — mission run ID, sequence, expected revision and signature **only**, never geometry.

All three must be encoded from a prepared editor proposal. `GET /api/v1/mission/draft` is handled separately by a future authenticated read/resync client and is intentionally **not** treated as a mutation acknowledgment by this codec.

## Limits and failure behavior

| Guard | Default |
|---|---:|
| Maximum encoded request body | 262,144 bytes |
| Maximum accepted successful reply body | 4,096 bytes |
| Maximum marking points | 10,000 |
| Maximum trusted elapsed request/reply age | 8 seconds |
| Success status | HTTP 200 only |

Reject a response rather than coerce incomplete values. The codec returns `valid=false` and has no physical or network side effects. Actual backend still owns authorization, operator lease, PX4 disarm verification, revision fencing, sequence checks and READY—not STARTED—mission transition.

## Tests

Four new Windows-compatible C++17 smoke tests:

- `backend_android_draft_codec_roundtrip_native_smoke`: real in-memory JSON encode/parse/decode over UPLOAD, UPDATE, LOAD, confirming READY and zero actuation.
- `backend_android_draft_codec_security_native_smoke`: prepared flag, trusted field, illegal path/method, sequence, identity, geometry, revision and input bound failures.
- `backend_android_draft_codec_ordering_native_smoke`: response correlation, authentication boundary, timeout, status, content type, replay and non-actuation claims.
- `backend_android_draft_codec_tamper_native_smoke`: forged success/loaded/ready/code, integer overflow, injected extra/duplicate keys, trust flags and strict schema rejection.

Expected Windows Visual Studio 2022 Release/CTest: **120/120** native tests, up from 116/116. Local Linux GCC results and clean-patch validation must also pass before distribution. Windows test completion is for the team to confirm.

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

## Explicit production blockers

Actual authenticated TLS ingress and certificate validation, Android Kotlin/Compose HTTP transport and decoding integration, GET /draft resynchronization, HTTP parser/CSRF/host/origin hardening, authenticated PX4 disarm attestation, ROS2 XRCE-DDS and Jetson deployment remain separate engineering tasks. Native tests do NOT establish 20 mm positioning, realtime reliability, motor/encoder correctness or hardware safety. **Rover hardware actuation and production deployment remain unapproved.**
