# DYX Autonomous — Phase 2H4Z

## Offline SQLite mission-draft transaction journal prototype
**Prepared by Satheesh** · C++17 / XRCE-DDS 4WD marking rover

**Exact base commit:** `6d16845f92016e092a4451475e956086293b5866` on `Jetson_DDS_Upgrade`.

## Why this phase

The H4W–H4Y replay-fence paths durably reserve ingress nonces but do **not** journal a complete mission draft or the server's final mutation response. H4Z adds a separate SQLite-only **prototype transaction boundary** with a full canonical request record and optional opaque full draft snapshot. Its purpose is to test how failures are handled *before* attaching a real Android service or the live PX4/DDS system.

**This is NOT wired into `NativeBackendMissionUpload`, `NativeOfflineDraftRestartFenceSource`, a network server, or actual actuators.** In particular, the existing H4W/H4X fence must **not** independently reserve the same nonce when calling the H4Z prototype, or the request will fail by design. The lab's request and outcome fields come from the caller; it does **not** cryptographically authenticate them or validate the full mission JSON. It is not permission to enable remote motion.

## Changes

- `backend_draft_sqlite_mutation_journal_lab.hpp` — a standalone C++17 SQLite journal using the existing H4X `replay_fence` table, without creating a missing replay database.
- `draft_sqlite_mutation_journal_native_smoke.cpp` — new offline transaction, failure, recovery, and concurrency tests.
- `test/sqlite_lab/CMakeLists.txt` — four additional tests in the optional SQLite suite.
- This documentation.

### Journal states and semantics

1. **Admin provision:** Add journal schema to an already provisioned H4X-style database. Constructor never creates, fixes, or silently enrolls a database.
2. **Begin:** In one `BEGIN IMMEDIATE` transaction, validate session/epoch/nonce/monotonic clock against the existing `replay_fence`, reserve the new nonce, and record full canonical request bytes with `PENDING` status. Duplicate requests or any previous unresolved `PENDING` are denied.
3. **Conclude:** In a separate transaction, record either a complete opaque draft snapshot plus a 200 response (`PREVIEW` or `READY`, never `STARTED`), or a non-success response marked `DENIED`. `LOAD` may only be recorded as `READY`.
4. **Reopen/recover:** Existing committed result and snapshot can be inspected offline; an unfinished `PENDING` record survives reopen and **blocks additional mutation**. No automatic replay or automated decision is made after a crash.

The **begin and conclude steps are separate transactions**. Therefore, this is *not* an atomic transaction with the existing in-memory mission backend and is *not* exactly-once mission execution. If the process crashes after the backend acts but before `conclude`, the journal remains PENDING; a future authenticated reconciliation/rebuild process must resolve the real backend state, not blindly re-run the request.

### New optional SQLite tests

| Test | Behavior exercised |
|---|---|
| `draft_sqlite_journal_nominal_native_smoke` | UPLOAD/UPDATE/LOAD journal, snapshot and 200/422 outcome, duplicate sequence, correct PREVIEW vs READY |
| `draft_sqlite_journal_recovery_native_smoke` | Pending survives reopen, repeat denied, no new mutation until resolved; completed snapshot survives reopen |
| `draft_sqlite_journal_faults_native_smoke` | Missing DB/schema, forged identity/epoch, invalid route, stale clock, invalid outcomes, corrupt journal metadata |
| `draft_sqlite_journal_concurrent_native_smoke` | Eight independent SQLite connections attempt same nonce; exactly one reservation succeeds |

**Expected counts:** 144 existing Windows core tests (unchanged), **12 optional SQLite tests** (8 existing + 4 H4Z). The four new H4Z tests were compiled and run under GCC 14.2 with SQLite 3.46.1 on Linux in a standalone source-level test harness. Full 12-test repository integration and Jetson hardware validation remain to be done.

## Windows Git and standard C++17 regression

```powershell
cd "D:\4WD_ROVER WORKSPACE FOR GITHUB\DYX_4WD_JETSON_DDS"
if (git status --porcelain) { throw "Working tree is not clean" }
git pull --ff-only origin Jetson_DDS_Upgrade
if ($LASTEXITCODE -ne 0) { throw "Pull failed" }
if ((git rev-parse HEAD) -ne "6d16845f92016e092a4451475e956086293b5866") { throw "Wrong baseline" }
$Patch = "$env:USERPROFILE\Downloads\DYX_CPP17_RPP_PHASE2H4Z.patch"
if (-not (Test-Path $Patch)) { throw "Missing patch" }
git apply --check $Patch
if ($LASTEXITCODE -ne 0) { throw "Patch check failed" }
git apply $Patch
if ($LASTEXITCODE -ne 0) { throw "Patch application failed" }
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
if ($LASTEXITCODE -ne 0) { throw "Configure failed" }
cmake --build build/dyx_cpp_only --config Release
if ($LASTEXITCODE -ne 0) { throw "Build failed" }
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Core regression failed" }
```

## Optional: run full SQLite tests on Windows via WSL2 Ubuntu

This is optional **without a Jetson** and is recommended to finish as much host-side work as possible. Requires a working WSL2 Ubuntu installation (or an Ubuntu VM) and a checkout of the same GitHub branch **inside Linux**.

```bash
sudo apt update
sudo apt install -y build-essential cmake libsqlite3-dev
cd ~/DYX_4WD_JETSON_DDS
git pull --ff-only origin Jetson_DDS_Upgrade
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B build/dyx_sqlite_lab -DCMAKE_BUILD_TYPE=Release
cmake --build build/dyx_sqlite_lab -j2
ctest --test-dir build/dyx_sqlite_lab --output-on-failure
```

Expected: **12/12** if the complete existing suite passes. Do not run the Linux SQLite path in PowerShell unless SQLite development libraries are installed and correctly linked.

## Production blockers / deferred hardware gate

- Implement a **real, audited integration** between authentication, request framing, nonce reservation, backend mission-draft state and the journal. Persist the full validated mission snapshot with a single recoverable state machine. Crash-consistency under real backend side effects is not implemented.
- Verify independent PX4 disarm/arming/mode/health evidence over production XRCE-DDS, safety inhibits, RC takeover, RTK and battery policy, and E-stop with motors physically isolated before any movement test.
- Test real Android HTTP/TLS client authentication, control-lease lifecycle, rate limits, credential rotation, and listener exposure; all currently remain outside this phase.
- Exercise SQLite power-removal, WAL/checkpoint, disk-full, corruption, permissions, trusted boot/clock epoch recovery and migration on the target Jetson filesystem. WAL/FULL is not by itself an entire power-failure proof.

**Decision:** Continue offline C++17 and WSL2/VM integration now. Keep robot actuation and Android network control disabled until the real safety and hardware validation gates pass.
