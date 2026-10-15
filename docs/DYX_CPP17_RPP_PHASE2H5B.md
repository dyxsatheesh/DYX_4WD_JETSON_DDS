# DYX Autonomous — Phase 2H5B

## Offline read-only lost-reply reconciliation for mission-draft journal
**Prepared by Satheesh** · C++17 / 4WD Marking Rover / XRCE-DDS migration

**Repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS` · branch `Jetson_DDS_Upgrade`
**Exact base commit:** `3e06d828b2c90f42eb6aa3e521b46258d2f5c4cf`

## Rationale

Phase 2H5A journaled UPLOAD/UPDATE/LOAD requests and quarantined a restarted in-memory mission backend. A lost network reply is not evidence that re-sending a POST is safe. This phase introduces an **offline, read-only reconciliation interface** for a future independently authenticated Android service to inspect a **single exact prior request**, without issuing a second mutation.

The new `NativeOfflineDraftReconciliationReadLab` demands a trusted evidence fixture (authenticated principal, read scope, fresh session, matching session ID and epoch), and an **exact match to the original canonical request JSON** before reporting a recorded outcome. These evidence booleans are **not real authentication**; only a future privileged transport adapter can set them after verification. No HTTP endpoint, HTTPS/TLS, DDS, Pixhawk access, backend restore, motor commands or spray/laser control is implemented here.

### Advisory outcomes

| Internal state | Meaning | Automation allowed? |
|---|---|---|
| `recorded_preview` | The journal recorded a 200 PREVIEW decision | **No**. Snapshot is still unverified for restore |
| `recorded_ready` | The journal recorded a 200 READY decision | **No**. READY is not START |
| `recorded_denial` | The journal recorded a backend rejection | **No** |
| `pending_operator_review` | The request was reserved but final outcome is unknown | **No. Never auto-retry** |
| `request_binding_conflict` | Canonical request doesn't match journaled bytes | **No** |
| `denied_access` | Session evidence or read permission insufficient | **No** |
| `storage_error` | Missing/unreadable row or uncertain journal state | **No, fail closed** |

**Every reply sets `is_verified_snapshot=false`, `can_restore_mission=false`, `can_start_or_resume=false`, `hardware_command_emitted=false`.** The API deliberately does not return the opaque recorded snapshot JSON, whose data has not been verified for safe recovery. A request with no matching journal row returns a generic failure rather than a permission to retry.

The current journal interface does not distinguish a missing record from database errors, so the adapter returns a generic 503. This avoids treating an I/O failure as proof that an earlier command never happened. This laboratory read interface should never block physical emergency stop, RC takeover or independent safety hardware.

## Files added/updated

1. `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_draft_reconciliation_read_lab.hpp` (new): scoped read-only reconciliation policy.
2. `src/dyx_rpp_cpp17_core/test/draft_sqlite_reconciliation_read_native_smoke.cpp` (new): four native tests.
3. `src/dyx_rpp_cpp17_core/test/sqlite_lab/CMakeLists.txt` (append): register four additional SQLite tests without removing the existing 16.
4. `docs/DYX_CPP17_RPP_PHASE2H5B.md` (new): this document.

## Validation

- The 4 new tests passed locally with GCC and SQLite, including close/reopen, pending, denial, cross-session isolation, mismatched request and missing-record failure.
- A **reconstructed 20-test suite** passed locally with the previous H4X/H4Y/H4Z SQLite source and **minimal mission-backend test stubs**; it is NOT equivalent to a full repo build.
- Run **all 20** against the real repository under WSL2 before committing. The Windows native-only regression should stay at **144 tests**.

Windows PowerShell:

```powershell
cd "D:\4WD_ROVER WORKSPACE FOR GITHUB\DYX_4WD_JETSON_DDS"
if (git status --porcelain) { throw "Dirty working tree" }
git pull --ff-only origin Jetson_DDS_Upgrade
if ($LASTEXITCODE -ne 0) { throw "Pull failed" }
if ((git rev-parse HEAD) -ne "3e06d828b2c90f42eb6aa3e521b46258d2f5c4cf") { throw "Unexpected baseline" }
$Patch="$env:USERPROFILE\Downloads\DYX_CPP17_RPP_PHASE2H5B.patch"
if (-not (Test-Path $Patch)) { throw "Missing patch" }
git apply --check $Patch
if ($LASTEXITCODE -ne 0) { throw "Patch verification failed" }
git apply $Patch
if ($LASTEXITCODE -ne 0) { throw "Patch application failed" }
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
if ($LASTEXITCODE -ne 0) { throw "Windows configure failed" }
cmake --build build/dyx_cpp_only --config Release
if ($LASTEXITCODE -ne 0) { throw "Windows build failed" }
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Windows regression failed" }
```

WSL2 Ubuntu (inside Ubuntu, not Windows PowerShell):

```bash
cd '/mnt/d/4WD_ROVER WORKSPACE FOR GITHUB/DYX_4WD_JETSON_DDS'
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```

Expected: **144 Windows tests** and **20 optional SQLite tests**. Do not commit on failures.

## Remaining production gates

1. Complete authenticated HTTPS/mTLS endpoint, principal/lease verification, session and boot-epoch lifecycle, and strict canonical request derivation from the verified ingress.
2. Add an independently validated, lossless persisted mission-state restore protocol. H5B **does not** restore the backend, does not verify the whole snapshot for restart, and **never permits** automatic restart.
3. Audit SQLite WAL/power-loss/corruption/disk-full behavior, concurrency and in-memory backend/state transactional consistency.
4. Test target Jetson Orin Nano, Pixhawk 6X XRCE-DDS, fresh GNSS/RTK/encoders/safety state, RC override, emergency stop and real actuation **with motors physically inhibited first**.

The standalone physical STOP pathway must never be gated by this journal or by recovery inspection.
