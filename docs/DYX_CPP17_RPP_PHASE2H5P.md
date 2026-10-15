# Phase 2H5P — Canonical fail-closed recovery HTTP ingress

**Prepared by Satheesh — DYX Autonomous | 15 October 2026**

## Purpose

This patch consolidates offline recovery HTTP routing into a single method owned
by `NativeOfflineRecoveryStartupOwnerLab`. It operates on the exact startup-owned
and quarantined C++17 mission gateway and uses H5O's existing separately
verified, read-only SQLite recovered-draft review. It prevents a future caller
from inadvertently routing upload/update/load operations to the mutable
`NativeBackendUploadHttpWire` while the backend is in recovery mode.

## Changes

- New `handle_recovery_ingress(input, independently_verified_review)` on the
  startup owner: explicit route handling for mission/status/live, mission
  commands, read-only recovered draft, and blocked upload/update/load.
- Every authenticated upload/update/load or ordinary draft endpoint returns
  HTTP 423 / `RECOVERY_QUARANTINE_COMMANDS_LOCKED` regardless of any claimed
  PX4 disarm flag. Unauthenticated mutations return 401.
- Only `GET /api/v1/mission/draft?mission_run_id=...` enters the existing
  SQLite re-verification path. Unauthorized, forged, invalid or tampered review
  access remains denied.
- Mission commands use the SAME H5L-quarantined owner gateway. Authenticated
  STOP produces software stop intent only, not a confirmed PX4/motor stop.
- Unknown routes return 404; new tests verify 423, 401, 403, 404, 405,
  read-only preview, persisted tamper lockout, stop and retired owner.
- The old H5O internal entry methods are retained for compatibility; a real
  service must use the new canonical entry and not expose the old direct paths.

## Validation

Apply only to clean baseline GitHub commit
`45b3345cbb32d80d575006958f49407ea91e1444`.

Windows PowerShell (actual Windows checkout):

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Ubuntu WSL2 terminal (NOT PowerShell):

```bash
set -e
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" -R '^draft_sqlite_recovery_h5p_' --output-on-failure
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```

Expected based on previous passing baseline: Windows **160**; Ubuntu SQLite **115**
(previous 106 plus 9 new); combined **275**.

## Critical deployment limitations

This is still **offline C++17 SQLite laboratory code**. It does not implement or
verify a Jetson systemd service, live HTTP listener, trusted authentication,
monotonic server clock, actual Android APK, XRCE-DDS bridge, Pixhawk/PX4
acknowledgement, motor/Roboclaw control, a physical STOP, independent RC
handover or real-world rover precision. CTest success does NOT confer
production safety approval. Use motors physically inhibited during subsequent
hardware integration. A real service must resolve dependencies and route
binding independently before accepting any commands.
