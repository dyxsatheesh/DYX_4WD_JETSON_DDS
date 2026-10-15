# DYX Autonomous — Phase 2H5E

**Versioned SQLite PREVIEW checkpoint for strictly read-only operator review**

**Prepared by Satheesh**
**Repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS` (`Jetson_DDS_Upgrade`)
**Verified remote baseline:** `0e99f93bdf0029a7f9080059f9a93e90ceb3cad6`
**Host environments:** Windows Visual Studio 2022 (C++17), Ubuntu 22.04 WSL2 with SQLite.

## Purpose

After H5D has inspected the *whole* persisted mutation chain, H5E can write a **version-1 immutable PREVIEW checkpoint** to the same SQLite WAL database and later retrieve a verified copy for a non-actuating operator map review, including after Jetson/host application restart. This is an opt-in OFFLINE experimental component. It is not a production recovery service.

`NativeOfflineDraftPreviewCheckpointLab::seal_preview(trusted_identity)` acquires SQLite's **BEGIN IMMEDIATE** write lock, runs H5D history screening inside that locked interval, rechecks the fence and final committed PREVIEW event, decodes/re-encodes the canonical snapshot, then stores a one-row versioned checkpoint. Repeating with precisely the same contents is idempotent. Updates after checkpoint creation cannot silently replace the existing checkpoint; explicit authorized retirement/archiving will need a future design.

`inspect_preview(trusted_identity)` opens SQLite strictly **read-only**, screens history, verifies the stored version/identity/nonce/revision/signature and byte-identical canonical snapshot against the latest journal event, and returns a copy of `PreparedPathSnapshot` **only for display/review**. On stale snapshots, PENDING/DENIED history, READY/load events, mismatched identity, invalid version, corruption or storage failures, it returns a non-success result.

**Safety boundary:** the checkpoint does not reconstruct `NativeBackendMissionUpload` or `NativeMissionCommandGateway`. It does not unquarantine H5A, perform replay/LOAD, establish real authentication, retrieve PX4 telemetry, or actuate motors, lasers or spray. The result's `can_restore`, `can_load`, `can_start_or_resume`, `hardware_command_emitted` are always **false**. SQLite content signatures check consistency only: no publisher authentication or tamper-resistant MAC/key management. A malicious administrator who can rewrite the complete database remains outside this prototype's threat model.

**STOP/RC/E-stop independence:** this offline checkpoint must never gate hardware STOP, RC takeover, physical E-stop, or safety watchdogs.

## Changed files

- `include/dyx_rpp_cpp17_core/backend_draft_preview_checkpoint_lab.hpp` — new offline capture/inspection API, `BEGIN IMMEDIATE` snapshot freeze, version=1 immutable record.
- `test/draft_sqlite_preview_checkpoint_native_smoke.cpp` — six host-only scenarios.
- `test/sqlite_lab/CMakeLists.txt` — add the separate six-test SQLite lane.
- `docs/DYX_CPP17_RPP_PHASE2H5E.md` — this guide.

## Six optional tests

| Test suffix | What it establishes |
|---|---|
| `nominal` | Journal PREVIEW sealed, read verified, repeated seal idempotent |
| `reopen` | New process can read checkpoint without reconstructing/starting backend |
| `stale` | UPDATE after checkpoint makes it stale and cannot overwrite it |
| `ready` | LOAD means READY, not a safe PREVIEW-recovery candidate |
| `pending` | Unresolved request blocks checkpoint inspection and recapture |
| `tamper` | Corrupt saved revision and false identity do not pass inspection |

## Windows validation (existing native C++ core)

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected core results: **144/144** (unchanged; H5E is in the separate SQLite lane).

## Ubuntu WSL2 validation

```bash
cd '/mnt/d/4WD_ROVER WORKSPACE FOR GITHUB/DYX_4WD_JETSON_DDS'
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```

Expected SQLite results: **35/35** (29 existing + 6 H5E). This requires validation against the user's full checkout. Source-development testing uses simplified synthetic backend support and does **not** validate real Jetson/PX4 devices.

## Next real production gates

1. Authenticated operator credentials, lease, ownership and fresh PX4 disarm/stopped proof from independently verified hardware telemetry.
2. Explicit checkpoint retirement/versioning and durable single-source-of-truth atomicity with journal and in-memory backend.
3. Full trusted PREVIEW-only backend reconstruction under new host epoch; never automatically resume READY/STARTED motions. Persist sequence/replay fencing and run-ID retirement across reboot.
4. ROS2/XRCE-DDS/PX4 integration; independent watchdog, battery/RTK fail-safe and physical motor/spray inhibit and STOP/RC takeover tests.
5. Jetson filesystem power-cut tests with physical actuators disconnected.
