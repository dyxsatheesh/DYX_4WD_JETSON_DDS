# DYX Autonomous — Phase 2H5H

## Atomic offline epoch rotation into durable mission-write quarantine

**Prepared by Satheesh**
**Project:** DYX Autonomous 4WD Rover — Jetson Orin Nano / Pixhawk 6X
**Repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS` / `Jetson_DDS_Upgrade`
**Required upstream commit:** `ecd1c9d02217bbcbe336277d8790c5dd545a7c04`

### Purpose

Phase H5A–H5G persisted mutation history, verified a saved PREVIEW, checked recovery requirements and built a **detached, read-only** mission review model. Phase H5H introduces the first **write-side** recovery step, but ONLY as an **offline safety quarantine**, not as permission to restore or execute a rover mission.

With a previously sealed, valid PREVIEW, H5H performs a single SQLite `BEGIN IMMEDIATE` transaction that (1) checks there is no epoch barrier, (2) runs H5F and H5G verification against the saved journal, (3) writes a permanent `QUARANTINED` barrier record and (4) advances the durable replay-fence epoch and generation **atomically** while preserving the last nonce and host timestamp. If any step fails, the transaction rolls back, leaving the original fence unchanged.

**The new epoch does NOT authorize new commands.** The H4Z journal's `begin()` and `conclude()` now reject writes as long as the durable epoch barrier exists, including requests forged with the newly incremented epoch. The H5A journaled adapter also remains a restart-locked prototype. H5H has **no API for removing the barrier**, restoring the command backend, LOAD, START, RESUME, motor, spray, laser or DDS command emission.

### Safeguards and limitations

- Existing mission backend must have no draft; passed mission gateway must have no active mission. A fresh backend/gateway pairing is expected during offline recovery rehearsal.
- H5F operator review, disarmed, stationary and no-active-mission inputs are **boolean laboratory claims**. They are **not** identity proof or PX4 attestation. Never source these values from Android payloads or accept this lab as a privileged production API.
- Old epoch requests are rejected because their epoch no longer matches the replay fence; new epoch requests are rejected because the independent write barrier is present.
- The last committed mutation nonce is never reset to zero, and the original journal/checkpoint is not erased.
- Schema provisioning creates the new `dyx_mission_recovery_epoch_barrier` table. Already-provisioned historical databases lacking this table must be migrated **explicitly and offline** by a privileged administrator using a reviewed backup and migration procedure. No automatic migration on mission ingress is permitted.
- H4Z rejects opening missing or malformed new schema. H5H intentionally does **not** implement unquarantining; that requires a later audited transition design.
- SQLite `synchronous=FULL` with WAL is used, but durability depends on the target filesystem, fsync implementation, power-loss behavior, and backup discipline; host tests are not power-cut certification.
- No crypto authenticity: stored snapshots and epoch barrier are not authenticated. The database must be protected from hostile/privileged modification separately.
- Preserving a draft PREVIEW does not imply permission to START/RESUME. STOP and RC takeover must always be independent of any recovery path.

### Changed files

| File | Purpose |
|---|---|
| `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_draft_epoch_quarantine_lab.hpp` | New atomic quarantine/epoch transaction. |
| `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_draft_sqlite_mutation_journal_lab.hpp` | Add barrier schema and deny journal `begin()` / `conclude()` after quarantine. |
| `src/dyx_rpp_cpp17_core/test/draft_sqlite_epoch_quarantine_native_smoke.cpp` | Eight offline native SQLite scenarios. |
| `src/dyx_rpp_cpp17_core/test/sqlite_lab/CMakeLists.txt` | Register eight additional tests. |
| `docs/DYX_CPP17_RPP_PHASE2H5H.md` | Team review documentation. |

### Acceptance tests

| Scenario | Verify |
|---|---|
| `nominal` | Saved epoch 3 → quarantined epoch 4; nonce and timestamp preserved, generation incremented; no command output. |
| `restart` | Barrier persists across journal reopen, and both old-epoch and new-epoch posts are rejected. |
| `authorization` | Operator/safety/epoch claims fail shut; a nonempty active backend is refused. |
| `stale` | A post-checkpoint UPDATE invalidates old PREVIEW and prevents rotation. |
| `pending` | A PENDING mutation prevents transition. |
| `ready` | A LOAD-to-READY journal prevents PREVIEW quarantine. |
| `failure` | Injected SQLite write fault rolls back both the barrier and epoch change. |
| `schema` | Missing barrier schema is refused rather than implicitly bypassed. |

**Windows PowerShell**, from repo root:

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected **144/144**. The optional SQLite tests do not run in this Windows C++-only lane.

**Ubuntu WSL2**:

```bash
cd '/mnt/d/4WD_ROVER WORKSPACE FOR GITHUB/DYX_4WD_JETSON_DDS'
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```

Expected **58/58** (50 existing + 8 H5H). Do not commit on test failure.

### Verification performed while preparing patch

- H5H CMake, file-integration and Git patch applicability checked against a reconstructed source baseline matching the upstream H5G CMake and original H4Z header by length/context.
- All **58/58 SQLite tests passed** in a local reconstructed Linux harness, but that harness substitutes simplified headers for some production mission backend dependencies. **This is not a claim of testing the exact full GitHub checkout.** User's Windows and WSL2 results are required before pushing.

### Next software step

Design a separate, audited **PREVIEW-only, controlled backend rehydration** workflow that consumes the quarantine barrier under a single durable writer, refuses all old nonces, and requires fresh operator authority. No START/RESUME, actuator release or live PX4 control is authorized by H5H. Hardware integration and actual power interruption tests are deliberately reserved for the later Jetson/PX4 validation stage.
