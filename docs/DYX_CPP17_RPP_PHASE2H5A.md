# DYX Autonomous — Phase 2H5A

## Offline SQLite journal ↔ real C++17 mission-draft backend adapter
**Prepared by Satheesh**
**Project:** 4WD marking rover, Jetson Orin Nano, PX4 1.17.0, XRCE-DDS migration
**Repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS` / `Jetson_DDS_Upgrade`
**Exact Git baseline:** `4f1d0a926ceff2df20d1450047c15df0e184af40`

## Purpose and scope

The H4Z SQLite journal (Phase 2H4Z plus CMake hotfix) and the C++17 `NativeBackendMissionUpload` exist separately. Phase 2H5A introduces **an opt-in offline adapter** that runs an actual backend `handle()` call *between* SQLite `begin()` and `conclude()` for UPLOAD, UPDATE and LOAD. It serializes a backend-derived snapshot on successful operations and records backend outcome data. No physical hardware, network server or ROS/DDS operation is performed.

**This is not production-ready crash recovery.** The backend and `NativeMissionCommandGateway` still keep their mission state in memory. `journal.begin()` and `backend.handle()` and `journal.conclude()` are NOT one atomic transaction. If the process stops after backend mutation but before journal completion, `PENDING` persists and physical operation must stay inhibited pending manual/authoritative reconciliation. The adapter NEVER automatically replays an uncertain request.

## Files changed

- `include/dyx_rpp_cpp17_core/backend_draft_journaled_upload_lab.hpp` (new): offline journal/backend adapter, canonical request binding, backend-derived draft snapshot serialization, fail-closed/quarantine policy.
- `include/dyx_rpp_cpp17_core/backend_draft_sqlite_mutation_journal_lab.hpp`: adds read-only `has_history()` for safe cold-start quarantine.
- `test/draft_sqlite_journaled_backend_native_smoke.cpp` (new): four integrated host-only tests.
- `test/sqlite_lab/CMakeLists.txt`: registers the four optional new SQLite integration tests.
- This document.

## Important contract

The trusted caller must supply independently verified `BackendMissionUploadRequest` principal/lease/disarm/timestamp fields and a `DraftJournalLabProposal` whose canonical JSON is byte-for-byte generated for that exact typed request. Client input does not grant trust. The adapter calls H4Z `journal.begin()` to reserve nonce **once**. Do **not** place the separate H4W/H4X nonce reservation immediately in front of the same request or it will double-reserve/reject.

**Cold-start quarantine:** even completed historical journal entries mean the just-constructed in-memory backend has not been restored. The adapter refuses new writes if any journal entry already exists, if `PENDING` exists, or if its backend was already initialized outside it. This conservative choice prevents a restart from incorrectly treating old mission state as empty and permitting a fresh upload. The adapter also quarantines following LOAD to READY or an ambiguous journal completion failure. It provides **no unquarantine or restore method**.

**No physical authority:** standalone STOP, hardware E-stop and RC takeover must remain accessible outside this draft adapter. The journal adapter must never be used to gate emergency stopping. It does not independently implement authentication, safety interlocks, or motor/spray/laser commands. The existing backend's LOAD means **READY, not START**.

## Four optional tests

| Test | What it covers |
|---|---|
| `draft_sqlite_backend_nominal_native_smoke` | Actual `NativeBackendMissionUpload` UPLOAD→UPDATE→LOAD plus SQLite journal; PREVIEW/READY snapshot and no actuation |
| `draft_sqlite_backend_binding_native_smoke` | Reject changed canonical JSON, session identity, clock, route and untrusted disarm/auth evidence before backend |
| `draft_sqlite_backend_reopen_native_smoke` | Reopen a completed journal with a fresh empty backend; no unverified automatic restore; mutation blocked |
| `draft_sqlite_backend_pending_native_smoke` | Simulated committed PENDING after process loss; pending survives reopen and backend is not invoked |

### Commands: Windows PowerShell (same 144-core test lane)

```powershell
cd "D:\4WD_ROVER WORKSPACE FOR GITHUB\DYX_4WD_JETSON_DDS"
if (git status --porcelain) { throw "Dirty tree: stop" }
git pull --ff-only origin Jetson_DDS_Upgrade
if ($LASTEXITCODE -ne 0) { throw "Pull failed" }
if ((git rev-parse HEAD) -ne "4f1d0a926ceff2df20d1450047c15df0e184af40") { throw "Wrong baseline" }
$Patch="$env:USERPROFILE\Downloads\DYX_CPP17_RPP_PHASE2H5A.patch"
if (-not (Test-Path $Patch)) { throw "Patch absent" }
git apply --check $Patch
if ($LASTEXITCODE -ne 0) { throw "Patch check failed" }
git apply $Patch
if ($LASTEXITCODE -ne 0) { throw "Patch apply failed" }
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
if ($LASTEXITCODE -ne 0) { throw "Configure failed" }
cmake --build build/dyx_cpp_only --config Release
if ($LASTEXITCODE -ne 0) { throw "Build failed" }
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Windows tests failed" }
```

### Commands: WSL2 Ubuntu terminal — source path is under /mnt/d

```bash
cd '/mnt/d/4WD_ROVER WORKSPACE FOR GITHUB/DYX_4WD_JETSON_DDS'
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```

Expected if full checkout passes: **16 SQLite tests** (12 existing + 4 new). Re-running configuration is essential to register new tests. You may use `ctest --test-dir "$HOME/dyx_sqlite_lab_build" -R draft_sqlite_backend_ --output-on-failure` to isolate the new tests. No Jetson hardware is necessary for these host-only checks.

## Development verification and remaining gates

Local source syntax, SQLite transaction interaction and **four test scenarios passed using a stub of the mission backend**, GCC C++17 and SQLite. This is not proof that the new integration builds or passes against the full repository: that must be checked in WSL2 and on the user's native Windows regression suite prior to commit. Do not label the integration complete before those checks.

Still necessary for production:

1. Audited **complete validated mission-state restore** and checkpoint/reconciliation semantics, including the gateway's run-ID reuse and READY stage. Current adapter intentionally quarantines on restart.
2. Truly coordinated backend/journal durability and uncertainty resolution. Two separate transactions do not imply exactly-once semantics.
3. Real HTTPS/mTLS authentication and operator control lease, bounded input validation, signed permissions and key rotation.
4. Live XRCE-DDS Px4 state provenance, independent PX4 disarmed/stopped watchdog, external motor inhibit, RTK/battery failsafes and unconditional STOP/RC path.
5. Target Jetson filesystem/power-loss tests with motors and spray outputs physically disconnected.

**Decision:** Continue host-only C++17 integration on Windows/WSL. Do not attach this experimental adapter to network ingress or enable rover motion.
