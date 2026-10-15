# DYX Autonomous — Phase 2H5D

## Offline C++17 journal history screening before mission recovery
**Prepared by Satheesh**
**Project:** Production-oriented 4WD marking rover / Jetson Orin Nano / Holybro PX4 1.17 / XRCE-DDS
**Branch:** `dyxsatheesh/DYX_4WD_JETSON_DDS` — `Jetson_DDS_Upgrade`
**Exact base:** `0705ce805af424ed56568ba2174c2d0e56ff7c93`

### Why this phase

H5C verifies **one saved snapshot**. A correctly signed, canonical snapshot alone does **not** prove the surrounding UPLOAD → UPDATE → LOAD history is coherent. A crash may leave an unresolved PENDING, or a journal could have an invalid revision, reused sequence, unrelated run, or missing predecessor. Phase H5D screens the **entire** H4Z/H5A journal from a single read-only SQLite transaction. It does not restore anything.

### Changed files

- `include/dyx_rpp_cpp17_core/backend_draft_history_screen_lab.hpp` (new): strict offline read-only mutation chain screening. Opens an **existing** SQLite database with `SQLITE_OPEN_READONLY`, checks schema and replay fence, uses a consistent SQLite read snapshot, revalidates every committed snapshot with the H5C decoder and H5A serializer, and cross-checks route, session, monotonic sequence/nonce, previous revision/signature, and READY transition.
- `test/draft_sqlite_history_screen_native_smoke.cpp` (new): five offline regression modes.
- `test/sqlite_lab/CMakeLists.txt`: registers the five optional Linux/WSL2 tests **without replacing the prior 24 tests**.
- This document.

### Safety policy

| Journal history | H5D decision | Automatic restore / motion |
|---|---|---|
| No history | `no_saved_mutation` | No |
| Coherent UPLOAD/UPDATE ending PREVIEW | `consistent_preview_history` | **No** — review data only |
| Coherent LOAD ending READY | `ready_operator_review` | **No** — READY is not START |
| Any unresolved PENDING | `pending_operator_review` | No |
| Any recorded DENIED | `denial_operator_review` | No |
| Tampering, malformed chain, fence mismatch, >1024 entries | `inconsistent_history` | No |
| Wrong trusted session/epoch or unverified principal | `access_denied` | No |
| Missing DB, bad schema or SQLite errors | `storage_error` | No |

The result exposes a *non-authoritative summary* (run ID, final revision, path signature, counts). It contains NO route to restore a backend, no upload mutation, no ROS 2 publication, no PX4 command, no motor/spray/laser operation and no `can_start_or_resume` flag. Both `can_restore` and `can_start_or_resume` are **always false**, including for consistent PREVIEW/READY histories.

### Important limitations

1. These tests are HOST-ONLY; a caller setting `authenticated=true` is **not real authentication**. Never expose this lab directly to Android HTTP or use its result to grant authorization.
2. SQLite `quick_check`, WAL and path SHA-256 check **consistency**, not cryptographic authenticity of the database publisher. An attacker who can rewrite both the DB and its hashes can forge a plausible history. Independent provenance and operating-system file permissions remain essential.
3. Canonical request binding verifies the exact fixed H5A request prefix, identity, sequence, and UPDATE/LOAD predecessor revision/signature. The request's full marking-points body is NOT independently compared against every stored snapshot. Production recovery needs full typed request replay verification and a durable checkpoint/commit protocol.
4. This is still a prototype: no restore API, no gateway/state replay, no safe unquarantine, no real-time network boundary, no automatic recovery from PENDING. Hardware testing remains necessary.
5. A single committed DENIED causes review rather than assuming it had no side effects. This conservative policy is deliberate.
6. SQLite may create auxiliary shared-memory/WAL files as part of its normal operation; this code issues no mutation statement and does not intentionally modify the mission tables.

### Host verification

**Windows PowerShell** (from repository root):

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected: **144 core tests** (unchanged; H5D is SQLite-only).

**Ubuntu WSL2**:

```bash
cd '/mnt/d/4WD_ROVER WORKSPACE FOR GITHUB/DYX_4WD_JETSON_DDS'
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```

Expected: **29 SQLite tests** (24 existing + 5 new). The five new modes are `nominal`, `ready`, `pending`, `tamper` and `security`.

At patch preparation time, **29/29 passed in a reconstructed SQLite suite on Linux** with simplified underlying mock backend. The actual repository integration and Windows suite must be run by the team, as in prior phases.

### Next engineering milestone

Develop a versioned, independently authenticated **persisted mission checkpoint** coupled to journal commit, then a disarmed-only recovery entry point that reconstructs a safe PREVIEW state without loading or starting PX4. Audit fault-injection points and monotonic epoch binding. The live DDS/PX4, RTK, battery, RC takeover, E-stop, motor and spray gates remain blocked until target hardware validation.

**Decision:** Safe to continue offline host development. **Not approved for production motion or remote mission control.**
