# Phase 2H5Q — Recovery lifecycle, restart and concurrency hardening

**Prepared by Satheesh — DYX Autonomous | 15 October 2026**

## Scope

H5Q hardens the **offline** `NativeOfflineRecoveryStartupOwnerLab` first introduced
in H5N and extended in H5O/H5P. The same owner owns one quarantined mission
gateway and one SQLite read-only review guard.

- Add one **per-instance recursive mutex** to serialize every public owner
  access that touches mutable gateway, recovery/SQLite guard, preview, status,
  command adapter, or retirement. The recursive form is necessary because
  canonical H5P ingress calls H5O's compatibility adapters inside the owner;
  the lock is not released between route selection and command execution.
- Keep gateway quarantine latched throughout repeated requests, simulated
  startup-owner destruction/reconstruction, attempted motion commands,
  repeated STOP intent, stale identity and tampered/replaced SQLite evidence.
- Retire remains one-way. In a concurrent retirement/preview race a preview
  may be approved *before* retirement wins the owner lock, but never after;
  motion always stays locked. STOP remains only **software intent**.
- Add ten new SQLite-native CTest registrations reusing the existing H5O test
  executable; no separate listener, Android app or hardware interface.

## Acceptance targets

Baseline GitHub commit: `7f046b526ec23ec765a9de400fb2ef7a66c97f4f`.
Windows native C++17 suite: **160/160** (unchanged).
Ubuntu/Jetson-style SQLite suite: **125/125** (115 previous + 10 new).
Total separately counted test cases: **285**.

H5Q test mode prefix: `draft_sqlite_recovery_h5q_`.

Windows PowerShell (after applying the patch):

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Ubuntu WSL2 shell (not PowerShell):

```bash
set -e
cd '/mnt/d/4WD_ROVER WORKSPACE FOR GITHUB/DYX_4WD_JETSON_DDS'
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" -R '^draft_sqlite_recovery_h5q_' --output-on-failure
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```

## Important boundaries

- Serialized access is only per `NativeOfflineRecoveryStartupOwnerLab` instance,
  not a cross-process file lock or evidence that multiple owners cannot race.
- The lock protects in-process state only. SQLite evidence and filesystem
  must still be trusted and independently verified at production boundaries.
- This does **not** wire up a Jetson systemd service, real HTTP network
  listener, real authenticated identity claims, trusted monotonic time,
  Android UI, XRCE-DDS, PX4, motors, real RC takeover, emergency stop,
  or a physical stationary-state confirmation.
- A 200 response from STOP means **software stop intent**, not proven
  physical braking. No hardware commands are emitted by the lab.
- Do not operate the rover from this test harness. Hardware acceptance and
  safe physical motor inhibition remain pending.
