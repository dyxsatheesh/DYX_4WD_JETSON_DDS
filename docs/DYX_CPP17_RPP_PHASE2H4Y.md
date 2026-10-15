# DYX Autonomous — Phase 2H4Y

## Offline SQLite-backed restart-fence integration checks
**Prepared by Satheesh** · C++17 / 4WD Rover Jetson DDS migration

**Exact baseline:** `fd5f460804bae367fbdd79a63dd0289356a472fd` on `Jetson_DDS_Upgrade`.

### Scope

This phase adds four tests using the real H4W `NativeOfflineDraftRestartFenceSource` with the H4X disk-backed SQLite `NativeDraftSqliteStoreLab`. It compiles through the actual repository's C++17 header interfaces instead of only the small interface-stub harness used in earlier development. The new tests are in the **optional** SQLite lane; existing 144 Windows native-only tests do not change.

The tests do not implement network, Android authentication, PX4 state attestation, DDS or actuator commands. Source evidence is still simulated. The SQLite store persists replay records but not complete mission state or exactly-once mutation outcomes.

### Added optional Linux/Jetson tests

| Test | Coverage |
|---|---|
| `draft_sqlite_fence_reopen_native_smoke` | Reserve nonce through H4W source; destroy source and SQLite connection; reopen; reject duplicate; accept next nonce |
| `draft_sqlite_fence_binding_native_smoke` | Reject changed session epoch, identity, transport trust and receive-clock rollback |
| `draft_sqlite_fence_fault_native_smoke` | Fail closed for missing DB, clock/session-source failures and deleted replay row |
| `draft_sqlite_fence_concurrent_native_smoke` | Eight independent SQLite connections compete to reserve one nonce; at most one succeeds and it must persist |

### Windows check (unchanged)

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
if ($LASTEXITCODE -ne 0) { throw "CMake failed" }
cmake --build build/dyx_cpp_only --config Release
if ($LASTEXITCODE -ne 0) { throw "Build failed" }
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Core tests failed" }
```

Expected Windows suite remains **144 tests**.

### Jetson/Linux optional SQLite suite

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libsqlite3-dev
cd ~/DYX_4WD_JETSON_DDS
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B build/dyx_sqlite_lab -DCMAKE_BUILD_TYPE=Release
cmake --build build/dyx_sqlite_lab -j2
ctest --test-dir build/dyx_sqlite_lab --output-on-failure
```

Expected SQLite lane **8 tests**, up from 4. Run these against the **full GitHub checkout** on your Jetson, not the interface-stub harness. This is required before claiming cross-module integration passed.

### Production blockers

1. A durable, atomic journal linking ingress request identity, mission mutation results and full backend state — SQLite nonce persistence alone does **not** imply exactly-once actions.
2. Genuine authenticated HTTPS, independent operator session/lease verification, cryptographic key lifecycle and identity controls.
3. Fresh independently validated PX4 disarm/state evidence over live Jetson–PX4 DDS, with a separate hardware motor inhibit/RC emergency path.
4. Concurrent service/process restart, battery removal, filesystem failure, disk-full, WAL recovery and power-loss testing on the Jetson carrier with motor outputs isolated.
5. Trusted monotonic-clock / boot-epoch continuity or fail-closed re-enrollment after OS reboot.

**Keep remote motion/start controls disabled during this phase.**
