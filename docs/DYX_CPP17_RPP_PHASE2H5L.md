# DYX Autonomous — Phase 2H5L

**Core gateway and mission-upload recovery quarantine — real native C++17 boundary**
**Prepared by Satheesh**
**Repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS` — `Jetson_DDS_Upgrade`
**Required baseline:** `4bc0402c204c4b26998257dd4c5d68decec2a1fd` (H5K)

## Motivation

Phase H5K gave a PREVIEW-only wrapper, but a caller with direct access to `NativeBackendMissionUpload::handle()` could bypass it. H5L addresses that bypass **within the same command gateway instance** by adding a process-local, one-way recovery quarantine latch to the **actual** C++17 `NativeMissionCommandGateway` and gating the **actual** `NativeBackendMissionUpload` on this latch. H5K `begin()` now latches the gateway before exposing a read-only PREVIEW, even if the saved record review later fails. It checks for an already active mission or pending live draft *before* stopping to prevent misclassifying an unsafe restart.

## Gate behavior

- Once `latch_recovery_quarantine()` is called, gateway `load`, `dispatch` for START/PAUSE/RESUME/NEXT/SKIP, and `begin_new_disarmed_px4_epoch` reject. It is irreversible for that instance.
- `tick()` forces its motion/session gate false and its emergency-stop flag true; latching also requests a decision-side safe-stop immediately.
- The upload backend returns HTTP 423 on every `handle()` path and will not discard or expose an in-memory live draft preview while gateway quarantine is latched. An alternate backend bound to the *same gateway* also rejects.
- Authenticated STOP and unconditional `force_stop()` remain available independently of recovery. The software STOP intent is not a substitute for the physical motor/actuator stop.
- H5K continues to display immutable **SQLite** PREVIEW via its reviewed reader while live upload/command paths are locked. The H5H SQLite write quarantine remains separate and in effect.

## Limitations and production gate

This step closes direct-method bypasses only for objects that share the same latched gateway. A **new gateway constructed separately**, direct DDS publisher, other HTTP listener, legacy runtime, or any other actuator path is *not* made safe by this latch. Production integration must use exclusive owner construction, apply quarantine before opening listeners, route all mutation/mission controls through the guarded gateway, and place independent physical STOP/RC safety above it. Calls to non-threadsafe gateway state must be serialized; the added boolean latch does not make concurrent command and startup paths atomic. Process crash does not persist this in-memory latch; H5H's durable SQLite quarantine must be rechecked on every new process startup.

The offline H5K booleans are not authenticated PX4 measurements. No real HTTP server, ROS 2 DDS transport, hardware E-stop, or PX4 deployment was tested here. Accuracy, real-time performance and production safety are unverified.

## Files updated

| File | Purpose |
|---|---|
| `mission_command_gateway.hpp` | New one-way recovery lock at load, command, epoch and tick boundaries; STOP remains available |
| `backend_mission_upload.hpp` | Reject all upload/draft direct calls while gateway locked |
| `backend_draft_quarantined_boot_guard_lab.hpp` | Latch core before reviewing H5J evidence |
| `draft_sqlite_quarantined_boot_guard_native_smoke.cpp` | Update old direct-bypass test: blocked instead of permitted |
| `backend_recovery_quarantine_native_smoke.cpp` | Nine native host tests of real gateway/backend methods |
| `test/native_cpp_only/CMakeLists.txt` | Register nine Windows C++17-native tests |

## Windows Release tests

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
if ($LASTEXITCODE -ne 0) { throw "Configure failed" }
cmake --build build/dyx_cpp_only --config Release
if ($LASTEXITCODE -ne 0) { throw "Build failed" }
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Windows tests failed" }
```
**Target: 153/153 Windows C++17 tests** (144 previous + nine new).

## WSL2 SQLite regression tests

```bash
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```
**Target: 88/88 WSL2 SQLite tests** (H5K's `active` scenario now verifies the direct bypass is blocked).

**Combined expected: 241/241**. This is an expected user-workstation result, not a claim that production/hardware testing occurred. In this environment, representative SQLite checks use simplified backend dependencies. The full real native regression suite must be executed on the user's repository before commit.

## Next milestone

Hard-wire exclusive gateway and upload ownership into the real HTTP/Android ingress startup, enforce authenticated operator/session/lease and trusted sensor evidence, and keep STOP/RC independent; then proceed toward DDS process integration and deployment packaging. No automatic recovered mission execution.
