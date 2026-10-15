# Phase 2H5R — Offline Recovery-Only Service Admission and Shutdown Gate

**Prepared by Satheesh — DYX Autonomous | 15 October 2026**

## Baseline

Repository: `dyxsatheesh/DYX_4WD_JETSON_DDS`
Branch: `Jetson_DDS_Upgrade`
Required H5Q commit: `b934ad25a1f155cf6a7ba6f5548849b8aede9c46`

## Purpose

H5Q serialized requests to a quarantined startup owner. H5R adds a separate,
OFFLINE model of service admission **above** that owner to prevent any recovery
request from reaching the owner until a recovery-only startup gate is opened.
This is a model of future service boot ordering, not a production server.

## Design

`NativeOfflineRecoveryServiceLifecycleLab` is a final, noncopyable, nonmovable
C++17 owner of exactly one `NativeOfflineRecoveryStartupOwnerLab`. Its
constructor finishes the startup owner's quarantine process before any
recovery request can be admitted. No mutable gateway, backend or owner reference
escapes from the service wrapper.

States:

1. `admission_closed` — startup gate closed. Every attempted ingress returns
   HTTP **423** (`RECOVERY_SERVICE_NOT_ACCEPTING`) without calling the owner.
2. `recovery_only_admitted` — explicit `admit_recovery_only()` succeeds only
   after confirming the owner is quarantined, cannot start/load, and has no
   active mission/draft. The only request dispatch is through H5P's canonical
   recovery ingress. Normal mission operation never opens.
3. `permanently_closed` — one-way shutdown retires the owner and rejects all
   subsequent requests, including STOP, because there is no live listener;
   hardware emergency-stop must be independent.

Service lock serializes dispatch, admission and shutdown. In-flight processing
finishes before shutdown can return. An exception in request processing
permanently closes the offline gate; it must never implicitly resume mission
operation. Reopening requires constructing a new object and rechecking SQLite.

### Special safeguards

- START/PAUSE/RESUME/NEXT/SKIP and draft upload/update/load stay quarantined.
- STOP in the admitted state is a **software intent only**, not a physical stop.
- Recovered preview remains read-only and SQLite-verified on every request.
- A missing recovery DB does not enable mission operation; it can still permit
  restricted STATUS/software STOP when the recovery-only gate is open.
- Authenticated principal, host time, control lease, review identity and PX4
  disarm flags **must** originate from an independently trusted ingress, never
  be populated from untrusted Android HTTP JSON.
- HTTP 423 for an unadmitted service uses an already-supported status code in
  the existing C++17 response encoder. No socket/network response is emitted.

## Changes

1. New `backend_recovery_service_lifecycle_lab.hpp` isolated service gate.
2. Eleven offline service-lifecycle regressions in the existing SQLite recovery
   owner test executable (`service_boot_closed`, `service_open_status`,
   `service_locked_actions`, `service_preview`, `service_missing_db`,
   `service_auth`, `service_tamper`, `service_shutdown`, `service_restart`,
   `service_parallel`, `service_shutdown_race`).
3. Eleven CTest registrations in the SQLite lab CMake configuration.

## Acceptance tests

Windows PowerShell (MSVC):

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G 'Visual Studio 17 2022' -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

**Expected Windows:** 160/160 tests.

Ubuntu WSL2 on the **same patched repository**:

```bash
set -euo pipefail
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" -N -R '^draft_sqlite_recovery_h5r_'
ctest --test-dir "$HOME/dyx_sqlite_lab_build" -R '^draft_sqlite_recovery_h5r_' --output-on-failure
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```

**Expected WSL2:** 11/11 new H5R tests, 136/136 total SQLite tests.
Combined target: 296/296. The local test harness used a synthetic HTTP
interface for some existing upstream dependencies; real Windows/WSL2 tests in
your checkout remain mandatory. The patch was designed for the exact verified
H5Q file hashes. Do not commit or push unless both actual suites pass.

## Critical production limitations

**H5R is NOT a production Jetson service or an installed systemd unit.** It
opens no socket; implements no HTTPS, authentication, hardware attestation,
Android network adapter or monotonic-clock source; does not publish XRCE-DDS,
execute PX4 offboard control, assert motor disable, verify a physical E-stop,
or provide RC handover. `admit_recovery_only()` is an OFFLINE test action, **not
proof of external security admission**. In production, create a separate
privileged service adapter which authenticates facts independently and enforces
listener start/stop ordering; prevent alternate code paths to unquarantined
mission backends. Keep motors/spray inhibited throughout integration testing.

## Next integration gate (future phase)

Implement authenticated network ingress and real process boot wiring, followed
by Jetson systemd tests, PX4/DDS negative tests, physical outputs inhibited,
independent safety-stop verification and field validation. Do not infer
production safety or rover accuracy from the native unit-test counts.
