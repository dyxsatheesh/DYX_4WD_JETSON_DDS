# DYX Autonomous — Phase 2H5G

**Detached, read-only PREVIEW reconstruction for post-restart operator review**
**Prepared by Satheesh**
**Repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS` / `Jetson_DDS_Upgrade`
**Exact prerequisite baseline:** `a00ec1e996cf021a56c9d4202d684fc96979a27f` (Phase 2H5F)

## Purpose

H5F verified an H5E sealed PREVIEW and its persisted replay metadata but offered only a preflight result. H5G converts that *already reviewed* persisted PREVIEW into a **detached, immutable, in-memory read-only facade** for display after a process restart.

This is intentionally **not yet a restoration of `NativeBackendMissionUpload`**. The live command backend stays empty, and `NativeMissionCommandGateway` is never changed by the H5G code. No new operator lease, epoch, replay reservation, LOAD, START or RESUME is granted.

`NativeOfflineRecoveredDraftPreviewLab::reconstruct_for_review()`:

1. Rejects an already-active mission gateway or repeated reconstruction.
2. Opens the existing SQLite WAL database strictly read-only and pins a transaction.
3. Runs H5F recovery preflight (checkpoint, full-history screening, operator review, epoch proposal and caller-asserted safety predicates).
4. Confirms original session/epoch, last nonce, mission ID and revision match the persisted replay fence and latest COMMITTED PREVIEW event.
5. Strictly decodes the journaled snapshot and checks exact canonical H5A serialization.
6. Validates goal geometry using a **temporary** `NativeMissionInstallation` object. That object is immediately destroyed; `gateway.load()` and motion methods are never called.
7. Retains only an immutable memory-owned view with original manual/automatic mode, geometry, signature, revision and advisory replay metadata.

`handle()` exposes only a bound, authenticated, read-scoped `GET /api/v1/mission/draft`-style read response. Every UPLOAD, UPDATE, LOAD, unknown command route or replay attempt returns **HTTP-like 423** (`RECOVERED_PREVIEW_COMMANDS_LOCKED`). `review_snapshot()` returns a COPY only after the same identity/run authorization check; callers cannot mutate the facade's memory.

## Safety boundaries

- **Never** installs mission goals into a running gateway, actuates the rover, sprays, triggers a laser, starts DDS, or unquarantines H5A.
- H5F trusted booleans in the test fixture are caller assertions, **not independently authenticated operator/vehicle evidence**. A real production service must obtain those facts directly from trusted sources. Never accept them from Android JSON.
- Signatures are **consistency**, not publisher authentication or tamper-resistant storage proofs.
- No durable epoch rotation, replay fence ownership transfer or backend recovery has been implemented. H5G is an offline non-actuating development exercise.
- STOP, RC takeover, hardware E-stop and independent watchdogs must remain unaffected by this and all future recovery work.
- PREVIEW copy is read-only even when the saved final nonce/sequence indicates prior valid commands. These fields are *not* authorization tokens.

## Files changed

| File | Purpose |
|---|---|
| `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_draft_recovered_preview_lab.hpp` | Detached PREVIEW-only read facade, identity check, canonical validation |
| `src/dyx_rpp_cpp17_core/test/draft_sqlite_recovered_preview_native_smoke.cpp` | Eight dedicated SQLite scenarios |
| `src/dyx_rpp_cpp17_core/test/sqlite_lab/CMakeLists.txt` | Register the eight tests |
| `docs/DYX_CPP17_RPP_PHASE2H5G.md` | This report |

## Eight SQLite tests

| Scenario | Acceptance check |
|---|---|
| `nominal` | Reconstruct and display a valid committed PREVIEW with no hardware command |
| `restart` | Fresh empty backend is not changed; copy cannot mutate retained preview |
| `requests` | All mutation routes and replay attempts locked; forged readers rejected |
| `manual` | Original MANUAL mission mode preserved without loading the runtime |
| `stale` | Updated journal after checkpoint prevents reconstruction |
| `pending` | A power-loss PENDING journal mutation blocks reconstruction |
| `ready` | Loaded READY state and active gateway both block PREVIEW reconstruction |
| `missing` | No checkpoint, rejected operator review, stale epoch or safety claim blocks reconstruction |

## Host tests

Windows PowerShell, repository root:

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected **144 / 144** unchanged.

Ubuntu 22.04 WSL2:

```bash
cd '/mnt/d/4WD_ROVER WORKSPACE FOR GITHUB/DYX_4WD_JETSON_DDS'
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```

Expected **50 / 50** (42 earlier + eight H5G). Initial H5G development passed 50/50 Linux SQLite tests using a **reconstructed test harness with simplified backend and goal-installation substitutes**. The actual full repository, Windows regression and actual Jetson/PX4 hardware remain **unverified until separately tested**. Do not infer production readiness from a synthetic host harness.

## Remaining next software gates

- H5H: durable, crash-consistent session/epoch transition design with journal/fence authority; disallow old epoch messages by construction.
- Next: separately verified live operator/PX4 evidence and PREVIEW-only actual backend restore with full idempotency and explicit operator consent. Never restore READY/STARTED automatically.
- Integrate C++17 ROS 2 / XRCE-DDS, Android API, battery/RTK/STOP interlocks, deployment service and end-to-end host/simulation tests **before** hardware acceptance.
