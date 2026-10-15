# DYX Autonomous — Phase 2H5C

## Read-only validation of persisted C++17 mission snapshots
**Prepared by Satheesh**
**Repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS` · branch `Jetson_DDS_Upgrade`
**Required exact base:** `626367360f2cd18a4863473109edab85d2caba87`

### Purpose

H5B permits an authenticated (externally verified) service to learn whether a lost UPLOAD/UPDATE/LOAD reply was recorded. However, a H5B `PREVIEW` or `READY` label **does not** validate the bytes of the saved mission. H5C adds **offline read-only consistency checks** for the actual H5A-produced full mission snapshot JSON.

This adds an audit *before* designing a controlled recovery mechanism. **It does not recover/recreate a mission, bypass the H5A quarantine, or authorize START/RESUME/LOAD.** There is no HTTP listener, hardware drive path, ROS 2/DDS IO, spray or laser action.

### Implementation

- `backend_draft_snapshot_audit_lab.hpp` (new): deterministic-format decoder for saved H5A snapshot JSON, strict type/range/structure validation, recomputation of the real `compute_path_signature()` and byte-for-byte roundtrip through H5A's existing snapshot serializer; checks saved operator, run and stage against the journal and H5B's read-only identity/request-binding decisions. Returns diagnostics **only**, never restorable geometry or control authority.
- `draft_sqlite_snapshot_audit_native_smoke.cpp` (new): four SQLite-backed tests.
- `test/sqlite_lab/CMakeLists.txt`: append four new tests without changing existing 20 entries.

### Tests

| New test | Verifies |
|---|---|
| `draft_sqlite_audit_preview_native_smoke` | UPLOAD snapshot, path consistency, SQLite reopen; no implicit restore |
| `draft_sqlite_audit_ready_native_smoke` | LOAD recorded READY, **not STARTED** |
| `draft_sqlite_audit_tamper_native_smoke` | Reject altered geometry and inconsistent loaded/stage bytes |
| `draft_sqlite_audit_security_native_smoke` | Reject missing/changed identity, request mismatch and uncertain PENDING |

Expected after applying: **144/144** unchanged Windows C++17 core tests and **24/24** WSL2 SQLite tests (20 existing + 4 new).

The local development run compiled 24/24 and passed 24/24 tests using GCC, SQLite, and a **simplified backend / geometry-signature test shim**. This is NOT proof of full-repository integration or production cryptographic correctness. Run the real repository under WSL2 to validate the true mission signature algorithm and backend.

### Windows PowerShell — apply only to the stated base

```powershell
cd "D:\4WD_ROVER WORKSPACE FOR GITHUB\DYX_4WD_JETSON_DDS"
if (git status --porcelain) { throw "Working tree is not clean" }
git pull --ff-only origin Jetson_DDS_Upgrade
if ($LASTEXITCODE -ne 0) { throw "Pull failed" }
if ((git rev-parse HEAD) -ne "626367360f2cd18a4863473109edab85d2caba87") { throw "Unexpected base" }
$Patch = "$env:USERPROFILE\Downloads\DYX_CPP17_RPP_PHASE2H5C.patch"
if (-not (Test-Path $Patch)) { throw "Patch not found" }
git apply --check $Patch
if ($LASTEXITCODE -ne 0) { throw "Patch check failed" }
git apply $Patch
if ($LASTEXITCODE -ne 0) { throw "Apply failed" }
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
if ($LASTEXITCODE -ne 0) { throw "Windows configure failed" }
cmake --build build/dyx_cpp_only --config Release
if ($LASTEXITCODE -ne 0) { throw "Windows build failed" }
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Windows tests failed — do not commit" }
```

### Ubuntu WSL2 — run in the Ubuntu terminal (not PowerShell)

```bash
cd '/mnt/d/4WD_ROVER WORKSPACE FOR GITHUB/DYX_4WD_JETSON_DDS'
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```

### Commit only after both real-repository test suites pass

```powershell
git add src/dyx_rpp_cpp17_core/test/sqlite_lab/CMakeLists.txt
git add src/dyx_rpp_cpp17_core/test/draft_sqlite_snapshot_audit_native_smoke.cpp
git add src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_draft_snapshot_audit_lab.hpp
git add docs/DYX_CPP17_RPP_PHASE2H5C.md
git diff --cached --check
if ($LASTEXITCODE -ne 0) { throw "Staged diff failed" }
git diff --cached --quiet
if ($LASTEXITCODE -eq 0) { throw "No staged changes" }
git commit -m "test(jetson): audit persisted C++17 mission snapshots without recovery"
if ($LASTEXITCODE -ne 0) { throw "Commit failed" }
git push origin HEAD:refs/heads/Jetson_DDS_Upgrade
if ($LASTEXITCODE -ne 0) { throw "Push failed" }
git status
git log -1 --oneline
```

### Critical safety and trust limits

- The existing `SHA-256` path signature verifies **path-content consistency**, not authenticity. It does **not** cryptographically bind owner, epoch, run ID, mission mode, revision or load stage. An attacker who can edit the database can change unprotected metadata or recompute an unkeyed signature. Treat an audit success as **consistency**, not authorization.
- Reconciliation identity/lease/PX4 facts are provided by a simulated trusted adapter. No live credential verification or DDS/PX4 evidence is obtained here.
- Journal rows are inspected by separate reads rather than one snapshot transaction; production review must address concurrent changes/TOCTOU, transaction boundaries, filesystem protections and rollback attacks.
- H5A uses separate `journal.begin()`, in-memory backend mutation and `journal.conclude()` steps. Crash uncertainty is not resolved by H5C. Cold-start quarantine remains mandatory. No automatic restore of RUNNING or READY states.
- Hardware and production service gates remain: mTLS/authentication/lease; real DDS bridge; safe-stop independent of journal; battery, RTK and RC controls; Jetson filesystem/power-cycle validation with disconnected actuators.

**Next milestone:** design a durable authenticated mission checkpoint that binds mission metadata and path content, with explicit operator-assisted recovery only after externally verified PX4 safe/disarmed state. Do not use an audit success to enable motion.
