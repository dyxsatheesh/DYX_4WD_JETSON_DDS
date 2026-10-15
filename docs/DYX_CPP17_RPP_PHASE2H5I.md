# DYX Autonomous — Phase 2H5I

## Immutable quarantined PREVIEW handoff staging (OFFLINE, non-actuating)

**Prepared by Satheesh**
**Project:** DYX Autonomous 4WD marking rover — Jetson Orin Nano / Holybro Pixhawk 6X
**Repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS` — `Jetson_DDS_Upgrade`
**Required git baseline:** `48371620b2aa9c827a66b86145c763972603619c`

### Purpose

H5H atomically rotates a *persisted* session epoch into a **permanent SQLite `QUARANTINED` mutation-write barrier**. H5I provides an **offline immutable PREVIEW handoff record** to support later safe recovery design, without releasing that barrier, rehydrating a command backend or granting motion authority.

H5I uses `BEGIN IMMEDIATE`, obtains the existing H5H barrier and new replay fence, and verifies old→new epoch, nonce, fence generation, session, mission run, revision and signature. It then invokes a new **read-only quarantined-history screening mode** on the existing H5D canonical journal auditor. This audits all saved UPLOAD/UPDATE mutations against the original epoch, while checking they are bound to the *current* rotated fence and permanent H5H barrier. The existing ordinary H5D `inspect()` behavior is unchanged.

It independently checks the H5E sealed PREVIEW checkpoint against the final immutable journal snapshot byte-for-byte, decodes strict canonical PREVIEW geometry, re-validates via the non-actuating `NativeMissionInstallation`, then saves one `STAGED` snapshot record in SQLite. Repeating the exact operation is idempotent; any mismatch fails shut. Post-quarantine tampering of saved history, checkpoint, barrier or staged record is rejected in tested scenarios.

**H5I never removes the H5H barrier, accepts mission commands, populates `NativeBackendMissionUpload`, invokes `gateway.load()`, sends DDS messages, or authorizes LOAD/START/RESUME, motors, spray or laser.** There is no unquarantine method. The returned geometry is **for operator review only**.

### Source changes

- `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_draft_quarantined_handoff_lab.hpp`: new PREVIEW staging API with all command-authority flags permanently `false`.
- `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_draft_history_screen_lab.hpp`: add strict, read-only `inspect_quarantined()` audit bound to H5H barrier. Ordinary `inspect()` unchanged.
- `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/backend_draft_sqlite_mutation_journal_lab.hpp`: require new immutable `dyx_mission_recovery_staging` schema and create it **only** through explicit administrator provision action.
- `src/dyx_rpp_cpp17_core/test/draft_sqlite_quarantined_handoff_native_smoke.cpp`: ten new SQLite smoke modes.
- `src/dyx_rpp_cpp17_core/test/sqlite_lab/CMakeLists.txt`: registers the ten Linux SQLite modes.
- This team note.

### Safety and operational limits

- **NO live authentication or PX4 attestation:** caller-supplied identity/safety booleans remain laboratory assertions. Never read those flags directly from Android, HTTP or DDS requests in production.
- **No unquarantine:** both old-epoch and rotated-epoch journal mutations remain rejected by H5H. Command gateways remain untouched.
- **No cryptographic integrity/authenticity:** SQLite contents are consistency-checked only. Privileged filesystem attackers can alter the file; full disk/role hardening and authenticated storage are pending.
- **Schema compatibility:** historical databases containing H5H but not the new H5I table must undergo a separately reviewed, offline administrative schema migration/back-up. Journal construction fails closed on a missing staging table.
- **Durability is conditional:** SQLite WAL + `synchronous=FULL` is not proof against SSD controller failure or physical power cuts. Test on the final Jetson storage later.
- **Independent E-STOP, RC takeover, STOP and safety latches** must remain untouched by any restoration workflow.
- This phase is a recovery *staging* mechanism, **not** production backend/rover recovery; the next software steps must integrate real C++17 backend ownership and privileged operator authority without permitting stale commands or automatic motion.

### Acceptance scenarios

| Mode | Expected result |
|---|---|
| `nominal` | Re-audit saved PREVIEW post-H5H; stage immutable copy; all authority flags false; journal remains locked. |
| `restart` | Exact re-staging returns idempotent response and old/new epoch requests are still rejected. |
| `auth` | Deny false operator/safety inputs and mismatched epoch/session. |
| `missing` | Refuse staging before H5H barrier has been committed. |
| `tamper` | Detect altered sealed checkpoint metadata. |
| `history` | Reject noncanonical/tampered journal command history. |
| `barrier` | Reject altered quarantine nonce/fence binding. |
| `failure` | SQLite INSERT failure rolls back; subsequent fresh attempt can succeed. |
| `conflict` | Refuse tampered/conflicting previously staged record. |
| `active` | Refuse handoff when backend already has draft. |

### Tests — host software only

**Windows PowerShell** (existing suite):

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected **144/144** (H5I is in optional Linux/Jetson SQLite lab).

**Ubuntu WSL2**:

```bash
cd '/mnt/d/4WD_ROVER WORKSPACE FOR GITHUB/DYX_4WD_JETSON_DDS'
cmake -S src/dyx_rpp_cpp17_core/test/sqlite_lab -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/dyx_sqlite_lab_build" -j2
ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
```

Expected **68/68** (58 existing + 10 new). **Do not commit if either suite fails.**

### Preparation verification and scope of confidence

Patch produced and checked against a reconstructed exact H5H changed-file baseline; H5D H5H headers and CMake sizes compared against remote `4837162`. All **68/68** Linux SQLite smoke modes passed locally in a reconstructed source suite; that suite uses simplified backend headers for some dependencies. It is **not** a complete real GitHub checkout, Jetson build, PX4 integration or safety qualification. Your host test results and later Jetson + PX4 hardware tests remain required.

### Next planned work

A controlled fresh-backend PREVIEW adoption boundary must keep quarantine latched and avoid treating the saved mission as LOADED or runnable. Subsequent software integration will require durable single-writer ownership, privileged operator verification, fault injection, ROS 2/DDS integration, and platform packaging. No recovered PREVIEW may automatically execute.
