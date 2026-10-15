# Phase 2H5O — Owner-bound offline HTTP recovery ingress

**Prepared by Satheesh — DYX Autonomous**
**Repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS`
**Branch:** `Jetson_DDS_Upgrade`
**Required parent:** `1ea8a06d17850d6f11e71dc44815c5b9a53021b5`

## Purpose

Bind H5M's existing **typed HTTP/JSON dispatcher** to the **same privately owned quarantined gateway** constructed by H5N. H5K's persisted recovered PREVIEW metadata has a dedicated, restrictive GET adapter. The owner creates no alternative gateway or exposed mutable backend. This makes it harder to accidentally couple recovery HTTP routing to a separate unlocked gateway.

## Changes

- `backend_recovery_startup_owner_lab.hpp`: adds `handle_http()` using the H5N-owned gateway for mission status and command routes; all motion commands remain locked (423). Authenticated STOP remains accepted as software intent only. Adds `handle_recovered_draft_http()` with explicit trusted operator/read scope, monotonic timestamp, matching review identity, exact body-free GET query, and **fresh SQLite review on every read**. SQLite/read faults cause permanent retirement; no coordinates or actuation commands serialized.
- `draft_sqlite_recovery_http_owner_native_smoke.cpp`: nine test scenarios for status, commands, STOP, valid PREVIEW, authorization, malformed requests, tamper, locked startup, and retirement.
- `test/sqlite_lab/CMakeLists.txt`: registers nine additional Ubuntu WSL2 CTests.

## Test target

| Lane | Before H5O | H5O added | Target |
|---|---:|---:|---:|
| Windows native C++17 | 160 | 0 | 160 |
| Ubuntu WSL2 SQLite | 97 | 9 | **106** |
| Combined host tests | 257 | 9 | **266** |

## Security and deployment limitations

**OFFLINE integration only — NOT production-ready.** This is not a running Jetson service, a bound HTTP socket or systemd startup integration. There is no real TLS, independent user authentication, monotonic ingress clock verification, hardware interlock, multi-process ownership, or ROS 2 XRCE-DDS/PX4 output. The typed request principal and independent review identity must be supplied by a trusted transport outside these classes, not Android JSON. H5O's `GET` output is metadata only; a valid PREVIEW **never restores or starts a mission**. STOP is a software decision, not evidence that motors or the Pixhawk stopped. All real actuator and DDS tests remain pending. Do not expose this lab directly to an untrusted network.

## Windows PowerShell

```powershell
cd "D:\4WD_ROVER WORKSPACE FOR GITHUB\DYX_4WD_JETSON_DDS"
if (git status --porcelain) { throw "Working tree is not clean" }
git pull --ff-only origin Jetson_DDS_Upgrade
if ($LASTEXITCODE -ne 0) { throw "Pull failed" }
if ((git rev-parse HEAD) -ne "1ea8a06d17850d6f11e71dc44815c5b9a53021b5") { throw "Unexpected baseline" }
$Patch = "$env:USERPROFILE\Downloads\DYX_CPP17_RPP_PHASE2H5O.patch"
if (-not (Test-Path $Patch)) { throw "Patch missing" }
git apply --check --whitespace=error $Patch
if ($LASTEXITCODE -ne 0) { throw "Patch check failed" }
git apply $Patch
if ($LASTEXITCODE -ne 0) { throw "Patch apply failed" }
git diff --check
if ($LASTEXITCODE -ne 0) { throw "Diff check failed" }
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
if ($LASTEXITCODE -ne 0) { throw "Configure failed" }
cmake --build build/dyx_cpp_only --config Release
if ($LASTEXITCODE -ne 0) { throw "Build failed" }
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Windows tests failed" }
```

## Ubuntu WSL2 (not PowerShell)

```bash
set -e
cd '/mnt/d/4WD_ROVER WORKSPACE FOR GITHUB/DYX_4WD_JETSON_DDS'
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" -R '^draft_sqlite_recovery_http_' --output-on-failure
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```

## Commit after both suites pass

```powershell
cd "D:\4WD_ROVER WORKSPACE FOR GITHUB\DYX_4WD_JETSON_DDS"
git add src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_recovery_startup_owner_lab.hpp
git add src/dyx_rpp_cpp17_core/test/draft_sqlite_recovery_http_owner_native_smoke.cpp
git add src/dyx_rpp_cpp17_core/test/sqlite_lab/CMakeLists.txt
git add docs/DYX_CPP17_RPP_PHASE2H5O.md
git diff --cached --check
if ($LASTEXITCODE -ne 0) { throw "Staged check failed" }
git diff --cached --quiet
if ($LASTEXITCODE -ne 1) { throw "Nothing staged" }
git commit -m "test(jetson): bind recovery HTTP requests to startup-owned gateway"
if ($LASTEXITCODE -ne 0) { throw "Commit failed" }
git push origin HEAD:refs/heads/Jetson_DDS_Upgrade
if ($LASTEXITCODE -ne 0) { throw "Push failed" }
git status
git log -1 --oneline
```
