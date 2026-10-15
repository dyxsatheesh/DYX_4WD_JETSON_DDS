# DYX Autonomous — Phase 2H4X

## Optional Jetson/Linux C++17 SQLite replay-fence persistence pilot
**Prepared by Satheesh** · 4WD Rover · C++17 / PX4 DDS migration

**Apply to:** `dyxsatheesh/DYX_4WD_JETSON_DDS`, branch `Jetson_DDS_Upgrade`, exact Phase 2H4W baseline `0cf3247d1d50042bb27b0c3aadd06c1ccfb35beb`.

### Purpose

Phase 2H4W proves the replay store interface with an in-memory mock. This phase adds an **opt-in, on-disk SQLite 3 implementation of that same interface** and four native tests. It allows the replay reservation to survive actual database close/reopen (unlike the prior mock). The standard 144-test Windows C++ suite is **unchanged**. This is a Jetson/Linux staging exercise, not Android network control or a sign-off for production security.

### New artifacts

- `include/dyx_rpp_cpp17_core/backend_draft_sqlite_store_lab.hpp`: SQLite 3 adapter implementing `DraftRestartLabStore`, using WAL + `synchronous=FULL`, `BEGIN IMMEDIATE`, verified current-record compare, and atomic update/commit. Missing/corrupt database or row denies. The constructor never creates or provisions a missing database.
- `test/draft_sqlite_persistence_native_smoke.cpp`: native SQLite policy/fault/recovery test executable.
- `test/sqlite_lab/CMakeLists.txt`: **separate optional build lane** requiring `libsqlite3-dev`.
- This document.

**Provisioning rule:** Database creation is an explicit out-of-band operation, `NativeDraftSqliteStoreLab::provision_new(path, trusted_initial_record)`; never invoke it from Android/HTTP request data. Its factory is a **lab-only convenience** and does not provide production-safe enrollment locking, path/ACL protection, encryption, or key management. Use a controlled local filesystem and review file ownership. Existing database files are not overwritten intentionally.

### Test matrix

| SQLite test | Check |
|---|---|
| `draft_sqlite_reopen_native_smoke` | Save nonce, close, reopen, verify saved record; reject duplicate provisioning and stale expected-record CAS |
| `draft_sqlite_conflict_native_smoke` | Two independent connections: stale compare-exchange rejected; refreshed record can advance |
| `draft_sqlite_faults_native_smoke` | Missing database, invalid provision, changed epoch, bad clock/nonce, missing record, corrupt database fail closed |
| `draft_sqlite_recovery_native_smoke` | Simulate lost response after committed reservation, reopen and reject replay; new nonce does not imply permission to repeat a mission |

### Windows — previous core tests (unchanged)

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
if ($LASTEXITCODE -ne 0) { throw "CMake failed" }
cmake --build build/dyx_cpp_only --config Release
if ($LASTEXITCODE -ne 0) { throw "Build failed" }
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Core tests failed" }
```

Expected **144/144**, not 148/148, since the SQLite tests use a **separate** opt-in lane.

### Jetson Ubuntu 22.04/24.04 — new SQLite tests

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libsqlite3-dev
cd ~/DYX_4WD_JETSON_DDS
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B build/dyx_sqlite_lab -DCMAKE_BUILD_TYPE=Release
cmake --build build/dyx_sqlite_lab -j2
ctest --test-dir build/dyx_sqlite_lab --output-on-failure
```

Expected separate SQLite result: **4/4 passing tests**. No rover connection or motors required. For Windows SQLite support, provide a working SQLite3 development package and configure this optional lane separately.

### Verified development checks

The four SQLite tests passed under GCC 14.2 and SQLite 3.46.1 on Linux in a **standalone interface-stub harness**, not against a full Jetson repository checkout. They validate the new SQL store logic and compile surface. The full cross-module runtime still requires Jetson integration validation.

### Production blockers — do not skip

1. Persist **mission mutation identity, mission state and outcome** in a coordinated crash-consistent transaction model. Persisting a nonce alone cannot guarantee exactly-once side effects, especially after backend process/power restart.
2. Audit SQL schema migrations, external file-system assumptions, disk-full/I/O errors, WAL/checkpoint behavior, directory/permission security, backup, storage wear and actual power-loss scenarios. SQLite FULL is not a universal hardware durability guarantee.
3. Resolve monotonic clock reset across Jetson reboot using trusted enrollment/boot epoch; fail closed on uncertainty. Test concurrent processes; the adapter uses an in-process mutex plus SQLite's transaction writer lock but is not an independently audited multi-service security boundary.
4. Wire real authenticated HTTPS, operator session/lease verification, live validated PX4 DDS disarm evidence, RTK/battery/RC safety arbitration, and external emergency stop. None are supplied by this patch.
5. Validate on actual Jetson Orin Nano 8 GB and Holybro Pixhawk 6X **with motors physically inhibited**, before considering any rover-motion enablement.

**Operational policy:** Keep the new SQLite test lane isolated from physical actuation and **do not** expose it as an Android command ingress or production credential store.
