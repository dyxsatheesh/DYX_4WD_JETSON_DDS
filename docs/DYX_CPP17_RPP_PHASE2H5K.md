# DYX Autonomous — Phase 2H5K

**One-time fail-closed startup guard for quarantined mission PREVIEW — OFFLINE C++17 laboratory**
**Prepared by Satheesh**
**System:** DYX Autonomous 4WD marking rover, future Jetson Orin Nano 8 GB / Pixhawk 6X integration.
**Repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS` / `Jetson_DDS_Upgrade`
**Required baseline:** `cde5bf16f2c51ee66774ca756ada4be834650550` (Phase 2H5J).

## Purpose

H5H saves a durable **QUARANTINED** epoch fence, H5I stages an immutable **STAGED** PREVIEW, and H5J verifies the saved PREVIEW strictly read-only. H5K introduces a **one-admission-attempt startup/session guard** wrapping the H5J reader and referencing the *real C++ backend and mission command gateway interfaces* (not constructing a runnable mission).

At startup, `NativeOfflineQuarantinedBootGuardLab::begin()` checks H5J's complete existing SQLite recovery evidence and checks that the gateway has no mission and the upload backend has no draft. A successful startup pins the operator session, epoch, mission run, signature, nonce, fence generation, revision and sequence **only to prevent identity drift**. Every subsequent read re-opens/rechecks full H5J evidence; nothing is rehydrated into the mission-control runtime. If the saved data changes, the backend has become active, or any H5J verification fails, the guard **retires irreversibly** for that process instance.

`handle()` deliberately **never forwards** a request to `NativeBackendMissionUpload::handle()`. Any upload/update/load/start/resume path returns `423 RECOVERY_QUARANTINE_COMMANDS_LOCKED`. Only a bound, authenticated, review-approved, body-free `GET /api/v1/mission/draft` can return descriptive PREVIEW metadata (`200 QUARANTINED_PREVIEW_READ_ONLY`). No command authority is granted. Unauthorized requests get no stored mission content.

## Safety properties and limitations

- Fail-closed initialization: only one `begin()` admission attempt per instance; failed boots cannot be retried on that instance.
- Read-only state is never a `LOAD`, `READY`, `START`, `RESUME` or replay-fence release.
- On each read, check *fresh* H5J SQLite evidence, operator/session epoch, immutable signature, nonce, revision, sequence and generation; on inconsistency, retire the guard.
- Attempts to use command routes, GET-with-body, sequence/revision/signature replay fields, or the wrong operator/run are rejected.
- Previously saved database writes remain prevented by the **independent H5H journal quarantine**; this lab does not weaken that barrier.
- **Critical integration limitation:** This is **not installed as the actual HTTP ingress**. A component that directly calls `NativeBackendMissionUpload::handle()` can bypass this wrapper. One test demonstrates this vulnerability by making an intentional direct call and then verifying that the guard detects the backend is no longer empty. The production ingress must provide **exclusive ownership and no bypass paths** before any motion-capable runtime can rely on the guard.
- The *emergency STOP and RC takeover must remain independent of this adapter and must never be blocked by its 423 behavior*. Do not route STOP through this lab adapter.
- The review claims (principal, read scope, epoch, evidence) are synthetic fixtures. No trusted auth, lease, live PX4 disarm/stationary proofs, OS permission enforcement or hardware control are implemented.
- SQLite consistency signature is **not cryptographic authenticity**; concurrent writers and adversarial database replacement remain separate security/integrity work. Do not treat successful host tests as hardware or production safety certification.

## Files changed

| File | Change |
|---|---|
| `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_draft_quarantined_boot_guard_lab.hpp` | New start-once, no-actuation guard exposing read-only PREVIEW and rejecting commands |
| `src/dyx_rpp_cpp17_core/test/draft_sqlite_quarantined_boot_guard_native_smoke.cpp` | Eleven offline SQLite host-test scenarios |
| `src/dyx_rpp_cpp17_core/test/sqlite_lab/CMakeLists.txt` | Register the eleven test cases |
| `docs/DYX_CPP17_RPP_PHASE2H5K.md` | This team note |

## Added test matrix

| Case | Verification |
|---|---|
| nominal | Valid boot + read-only geometry; no load or hardware emission |
| restart | Fresh guard instances can review stable quarantined records |
| uninitialized | No GET allowed before explicit startup |
| writes | UPLOAD/UPDATE/LOAD/START/RESUME all return locked |
| readonly | GET allowed; POST, payload and replay-parameter smuggling denied |
| identity | Wrong operator/run/epoch/authorization denied without poisoning good operator access |
| tamper | Changed durable staging locks and permanently retires guard |
| active | Direct mutation bypass is detected on next GET (demonstrates why exclusive ingress is mandatory) |
| failed_boot | Missing stage fails closed; second begin never succeeds |
| retirement | Explicit retirement cannot be reversed on same object |
| journal | Old and new epoch mutation attempts fail while quarantined |

## Windows test procedure

From the repository root, using Windows PowerShell:

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
if ($LASTEXITCODE -ne 0) { throw "Configure failed" }
cmake --build build/dyx_cpp_only --config Release
if ($LASTEXITCODE -ne 0) { throw "Build failed" }
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Windows tests failed" }
```

**Expected:** 144/144 Windows C++-only tests pass (unchanged native suite).

## WSL2 Ubuntu test procedure

```bash
cd '/mnt/d/4WD_ROVER WORKSPACE FOR GITHUB/DYX_4WD_JETSON_DDS'
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```

**Expected:** 88/88 SQLite tests (77 from H5J, 11 new in H5K). **Total expected host checks: 232/232.**

The offline development Linux harness passed 88/88. It uses a synthetic substitute for parts of the native mission upload backend; compile and run the user's *full real checkout* in WSL2 before commit/push. Never run physical rover actuation as a result of this patch.

## What remains

The next major work should **integrate these validated components into a real, exclusive production HTTP/backend ingress lifecycle**, establish authenticated identities and independently verified safety evidence, preserve STOP/RC precedence, then finish DDS integration and packaging. Avoid creating endless additional offline-only layers. Actual Jetson/PX4/field testing remains a separate later validation stage, not already completed.
