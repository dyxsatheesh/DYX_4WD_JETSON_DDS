# DYX Autonomous — Phase 2H5J

**Immutable-quarantine PREVIEW reader — offline C++17 laboratory**
**Prepared by Satheesh**
**System:** DYX Autonomous 4WD marking rover; future Jetson Orin Nano / Pixhawk 6X integration.
**Required Git baseline:** `3c0b3c9a3d525d2c3f7afa5f06928dbcbb2fee5c` on `dyxsatheesh/DYX_4WD_JETSON_DDS`, branch `Jetson_DDS_Upgrade`.

## Goal and exact trust boundary

H5H atomically rotated the SQLite replay-fence epoch into durable **QUARANTINED**; H5I sealed the immutable **STAGED** PREVIEW payload. H5J adds an **entirely read-only** observer that allows inspection of the staged PREVIEW, while requiring that both existing durable records and complete historic journal remain consistent. It does **not** restore `NativeBackendMissionUpload`, release the replay fence, replace a command gateway, create an HTTP listener, connect XRCE-DDS or act on motors, spray, or laser.

A fresh empty backend and inactive gateway are required. `NativeOfflineQuarantinedPreviewReviewLab::inspect()` opens an existing SQLite database in read-only/query-only mode, obtains a read transaction, validates H5H's epoch barrier and current replay fence, verifies all H5I staging metadata and exact canonical checkpoint payload, checks the last journal entry plus H5D full mutation history, and decodes/validates geometry using the native mission installation constructor. Only then does it return a **value copy** of the PREVIEW, mode and metadata for operator review. No SQLite writes occur.

The optional `handle()` model illustrates an offline, HTTP-style boundary: **423 Locked** for *every* command path including UPLOAD, UPDATE, LOAD, START and RESUME; a properly bound and reviewed GET of `/api/v1/mission/draft` may return PREVIEW metadata. It is **not connected to production HTTP or the Android application**. It does not return the full snapshot via the handle method.

**Important:** All booleans / identity claims in the laboratory API are test inputs, **not independently authenticated production facts**. Requiring them provides testable checks, not a trusted security boundary. The checkpoint signature is a consistency digest rather than a cryptographic database authentication tag. Full-on-disk integrity against an adversarial SQLite editor would require authenticated durable storage.

## Files

- `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_draft_quarantined_review_lab.hpp`: new query-only independent review component.
- `src/dyx_rpp_cpp17_core/test/draft_sqlite_quarantined_review_native_smoke.cpp`: nine Linux SQLite host-test scenarios.
- `src/dyx_rpp_cpp17_core/test/sqlite_lab/CMakeLists.txt`: registers nine new tests.
- `docs/DYX_CPP17_RPP_PHASE2H5J.md`: this team note.

## New test matrix

| Scenario | Expected outcome |
|---|---|
| nominal | STAGED+QUARANTINED PREVIEW/geometry/mode readable, command flags false |
| restart | Same immutable PREVIEW can be reviewed after reopen; journal writer remains locked |
| authorization | Reject missing access/review evidence, stale session epoch or forged operator ID |
| missing | No staged payload ⇒ no PREVIEW |
| stage_tamper | Modified staging sequence rejected |
| barrier_tamper | Modified quarantine generation rejected |
| history_tamper | Modified canonical mutation history rejected |
| active | Existing backend draft blocks recovered review |
| locked | POST/UPLOAD/UPDATE/LOAD/START/RESUME locked; bound GET only |

**Windows PowerShell (from repository root):**

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```
Expected baseline: **144/144**. H5J Linux-only test is intentionally not linked into the Windows C++-only lane.

**Ubuntu WSL2 (from repository root):**

```bash
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```
Expected: **77/77** (68 existing H5I plus 9 H5J). Test against the **actual full repository** before committing; a simplified development harness is not production verification.

## Safety and future development

The H5H quarantine is intentionally permanent from the APIs provided here. H5J cannot release it. Future phases must integrate a privileged source of operator identity/control lease and independently verified PX4 disarmed/stationary/no-active-mission status, isolate all mutation writers, and add authenticated storage/replay-fencing with failure-injection. No recovered PREVIEW may implicitly transition to a READY/RUNNING mission; LOAD and START require new explicit authority and independent safety gating.

This is a **host-software milestone**, not Jetson/PX4 hardware certification. Continue to test software on Windows and WSL2; reserve actuator-connected tests until an explicit safe hardware validation plan has been completed.
