# Phase 2H4R — C++17 Android Mission Draft GET/Resync Integration (OFFLINE)

**DYX Autonomous · Prepared by Satheesh · 15 October 2026**

## Purpose

Adds a transport-neutral, single-flight reconciliation contract for `GET /api/v1/mission/draft?mission_run_id=...`, filling the recovery gap after the 2H4O editor, 2H4P backend HTTP service, and 2H4Q Android draft mutation codec. **No listening POST socket is enabled.** No credentials, TLS, ROS 2, DDS or PX4 hardware are present in the offline unit-test lane.

The Android UI must **not retry POST UPLOAD/UPDATE/LOAD just because the response timed out**: a request can have succeeded on the server even when its response is lost. Instead, reconcile using an independently authenticated, session-owned `GET /draft` response and require a fresh preview/revision/signature before issuing another mutation.

## Changes

- `backend_android_draft_sync.hpp` — `NativeOfflineAndroidDraftSync` prepares one read-only GET request and accepts at most one verified reply per incrementing request ID. Failed, repeated, stale, untrusted or mismatched replies are rejected and consumed.
- `backend_android_mission_editor.hpp` — adds a mission-ID screen identity query so a reply for an old UI mission cannot be applied after switching to a different mission.
- The sync decoder only accepts the backend's exact canonical JSON schema, with strict revision, signature, point counts, ready/loaded/preview invariants; no JSON-supplied trust fields.
- A verified, exact HTTP 404 `NO_PENDING_DRAFT` can reconcile the editor to the *empty* state for initial upload. Arbitrary HTTP 404, HTTP 401/403, malformed/error responses, or unauthorized requests must not clear the draft.
- A verified HTTP 200 `DRAFT_PREVIEW_READY` or `MISSION_LOADED` informs the editor of an authenticated *server decision*, but does not prove physical control or grant START privileges.

## Required privilege boundary

`AndroidTrustedDraftSyncReply` contains verifier-provided metadata (authenticated transport, request ID/path, monotonic send/receive times). This is an **interface contract**, not cryptographic authentication. The eventual Jetson TLS adapter must independently validate certificates/credentials, authorization/ownership, request correlation and timestamps. It must never populate trusted metadata from Android JSON/HTTP headers or a client-declared `verified` flag. Backend still checks read scope and draft ownership. The existing loopback TCP listener stays read-only and unauthenticated; it is not a production Android network endpoint.

## New native tests

1. `backend_android_sync_nominal_native_smoke`: empty GET, upload, preview sync, update, new preview, LOAD and loaded GET, with READY—not STARTED—stage and no hardware emission.
2. `backend_android_sync_security_native_smoke`: invalid IDs, duplicate requests, spoofed transport claims, request mismatch, content type, oversized reply and forged hardware/network flags.
3. `backend_android_sync_recovery_native_smoke`: server accepts an UPLOAD but loses the reply; client times out and reconciles with GET, then rejects old-run UI replies and foreign-owner reads.
4. `backend_android_sync_timeout_native_smoke`: elapsed deadline, impossible trusted timestamps, false 404, replay, cancellation, monotonic clock rollback and restart-fencing behavior.

Expected total native Windows suite: **124 tests** (previously 120). Local GCC tests provide supplementary evidence, not Windows validation.

```powershell
cd "D:\4WD_ROVER WORKSPACE FOR GITHUB\DYX_4WD_JETSON_DDS"
$Patch = "$env:USERPROFILE\Downloads\DYX_CPP17_RPP_PHASE2H4R.patch"
if (-not (Test-Path $Patch)) { throw "Download Phase 2H4R patch first" }
git status --short
git pull --ff-only origin Jetson_DDS_Upgrade
if ($LASTEXITCODE -ne 0) { throw "Pull failed" }
git apply --check $Patch
if ($LASTEXITCODE -ne 0) { throw "Patch check failed" }
git apply $Patch
if ($LASTEXITCODE -ne 0) { throw "Patch apply failed" }
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
if ($LASTEXITCODE -ne 0) { throw "CMake configuration failed" }
cmake --build build/dyx_cpp_only --config Release
if ($LASTEXITCODE -ne 0) { throw "C++17 build failed" }
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Tests failed; do not commit" }
```

## Remaining production blockers

Actual Kotlin/Compose Android transport, authenticated TLS with authorization and certificate verification, true network ingress and security hardening, real Jetson ROS2/XRCE-DDS, fresh PX4 disarm attestation, actuator safety integration, real rover tests, and field accuracy validation have not been established. **This phase is not production deployable.**
