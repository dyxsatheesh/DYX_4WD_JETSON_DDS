# DYX Rover: Persistent Memory, Mission Recovery and Settings Storage Plan

**Status:** plan only. Nothing here is implemented.
**Date:** 2026-09-30.
**Builds on:** `docs/DYX_Mission_Power_Cycle_Recovery_Logic.md` (SHARINI). That doc defines the operator behaviour, and this plan keeps it. This plan adds the storage, identity, index, restore operation and restart handling needed to build it safely. It also covers where *every* setting is stored.

**Decided:**
- Recovery restores the mission as PAUSED, and the rover never moves after a reboot on its own.
- Resume uses the normal RESUME.
- A battery-critical event becomes a PAUSE and uses the same recovery.
- Out of scope: GNSS/RTK re-fix effects on recovered missions (operator decision, 2026-09-30).

Facts below were checked against the code on `latency-hardening` @ `21d4d4e`.

---

## Part A: Storage foundation (used by everything below)

### A1. Four kinds of data. Never mix them.

| Kind | What | Lifetime | Example |
|---|---|---|---|
| **Settings** | Operator/engineer choices | Until changed | spray duration, obstacle zones, voice, default marking tool |
| **Mission memory** | The loaded mission and its progress | Until the mission is stopped and cleared | mission.csv, point index, run id |
| **Safety journals** | Crash-safe records of physical actions | Until the action is reconciled | spray press journal |
| **Records** | History, never edited | Kept or rotated | mission reports, audit log, backups |

Settings export/import must never include mission memory or journals.

### A2. One data directory

Everything lives under the existing backend data directory, `~/.local/share/dyx_rover/` (`DYX_DATA_DIRECTORY`, `config.py`):

```text
~/.local/share/dyx_rover/
  settings/                 # Part H: one file per domain
    spray.json  mission_defaults.json  obstacle.json  voice.json  ...
  mission/                  # Part C
    active/                 #   the one active mission
      mission.csv           #   exact uploaded bytes
      mission_meta.json     #   identity (B2)
      mission_runtime.json  #   progress + index (C1)
    archive/                #   existing mission_archive (unchanged)
  journal/
    spray_journal.json      # moved from ~/.ros/, same content
  records/
    reports/  audit/settings_audit.jsonl  backups/
  backend.sqlite3  rtk/rtk.sqlite3   # unchanged
```

Existing `~/.ros/dyx_*.json` files are migrated once. On the first start after the upgrade:
1. Read the old file.
2. Write the new one.
3. Leave the old file renamed as `*.migrated`.

The path stays configurable through `DYX_DATA_DIRECTORY`.

### A3. Write rules (every persisted file)

1. **Atomic:** write to `name.tmp`, then `fsync(file)`, then `rename()`, then `fsync(directory)`. `mission_store._atomic_write_bytes` and obstacle `save_settings` already do this. Make it the only way to write.
2. **Envelope:** each file carries:

   ```json
   {"schema": "dyx.mission_runtime", "schema_version": 1, "revision": 42,
    "written_at": "...", "writer": "rover_backend", "boot_id": "...", "data": {...}}
   ```

   - `revision` increases by 1 on every write.
   - `boot_id` is `/proc/sys/kernel/random/boot_id` (used in Part E).
3. **Single writer per file.** Many processes can read a file, but only one owner ever writes it. The owner is listed in each section.
4. **Corrupt or unreadable file:**
   - Never crash, and never guess.
   - Move it to `name.bad.<timestamp>`.
   - Fall back to a safe result: defaults for settings, *no recovery offered* for missions.
   - Raise a health warning that the tablet shows.
5. **Write on change, not on a timer.** This limits SD/eMMC wear. Mission memory is written on state and point transitions only (C4).
6. **Shared library:** a new pure-Python package `dyx_persistence` provides atomic write, envelope, schema migration, corrupt handling and validation hooks.
   - Nodes (spray_controller, mission_manager, obstacle_monitor) and rover_backend all use it.
   - It's a new `ament_python` package, so it needs one `colcon build`. After that the symlink install applies.

---

## Part B: Mission stage and identity (item 2)

### B1. Mission stages

One authoritative stage, derived from Mission Manager state and persisted by the backend:

| Stage | Mission Manager state(s) | Persisted | Recovery after restart |
|---|---|---|---|
| `NONE` | EMPTY | nothing | none |
| `UPLOADED` | EMPTY/READY, never started | csv + meta | **no popup.** Mission reloads and re-prepares, same as today |
| `PREPARED` | READY, never started | csv + meta | no popup |
| `STARTED` | RUNNING, PAUSED, WAITING_FOR_NEXT | csv + meta + runtime | **popup: Resume / Stop & Upload New** |
| `TERMINAL` | COMPLETED, or stopped by the operator | runtime marked terminal | no popup. Report stays available |
| `CLEARED` | EMPTY after clear | active/ emptied | none |

`STARTED` begins at the first successful START (new `mission_run_id`). It ends only at COMPLETED, operator STOP, or "Stop & Upload New Mission". An ERROR while started stays `STARTED`, so recovery is still offered.

### B2. Mission identity: what makes "the same mission"

Stored in `mission_meta.json` at upload and never changed afterwards:

| Field | Source | Why |
|---|---|---|
| `mission_id` | mission_store | Primary key |
| `csv_sha256` | SHA-256 of the exact stored bytes (`mission_store._sha256`) | Detects any edit or replacement |
| `total_points` | mission_store `total_points` | Count check |
| `point_ids` | `P0001`…`Pnnnn` (Mission Manager's `f"P{n+1:04d}"`) | Index key |
| `point_latlon` | Surveyed lat/lon per point (from the CSV) | Lets the index be checked against the CSV |
| `extension_mode`, `coordinate_mode`, `dummy_point_distance_m`, `row_transition_threshold_m` | mission_store metadata | Same trajectory inputs |

⚠ **Do NOT use `path_signature` for identity.** `trajectory_generator._make_signature` hashes *local x/y*, which changes whenever PX4's local origin changes (every reboot). It stays a *within-session* check only.

**Identity check at recovery.** Every item must match, or recovery is refused with a stated reason:
1. `csv_sha256` of `active/mission.csv` equals the value in meta and in runtime.
2. `mission_id` matches in meta and runtime.
3. `total_points` and `point_ids` match between the rebuilt trajectory's marking points and meta.
4. The runtime `schema_version` is supported.

---

## Part C: Mission memory and the point index (item 3)

### C1. `mission_runtime.json` (owner: rover_backend)

```json
{
  "schema": "dyx.mission_runtime", "schema_version": 1, "revision": 57,
  "written_at": "2026-09-30T14:02:11+05:30", "boot_id": "…",
  "data": {
    "mission_id": "…", "csv_sha256": "…",
    "mission_run_id": "…", "run_started_at": "…", "recovery_count": 0,
    "stage": "STARTED",
    "state_at_write": "RUNNING",
    "pause_reason": null,
    "execution_mode": "MANUAL", "marking_tool": "LASER",
    "index": {
      "total_points": 9,
      "points": [
        {"id": "P0001", "status": "COMPLETED", "marking": "CONFIRMED", "completed_at": "…"},
        {"id": "P0007", "status": "ACTIVE",    "marking": "NONE"},
        {"id": "P0008", "status": "PENDING",   "marking": "NONE"}
      ],
      "active_point_id": "P0007",
      "resume_point_id": "P0007"
    },
    "last_known": {
      "raw_lat": 13.0, "raw_lon": 80.0, "fix_type": 6,
      "at": "…", "px4_armed": true, "px4_mode": "OFFBOARD"
    }
  }
}
```

### C2. Point index rules

- **Per-point status:** `PENDING`, `ACTIVE` (the point being driven to, or held at), `COMPLETED`, `SKIPPED`, `FAILED`.
- **Per-point marking evidence:**
  - `NONE`: nothing marked.
  - `CONFIRMED`: spray journal COMPLETED, or laser shown.
  - `UNCERTAIN`: see C3.
- **Resume point** = the first point that is not COMPLETED/SKIPPED/FAILED, in mission order. Never P1 unless P1 is unresolved.
- A point becomes `COMPLETED` only from Mission Manager's point event (the same event the report already stores). The index never infers completion itself.
- On recovery, `ACTIVE` becomes the resume point, and the rover drives to it again.

### C3. Spray journal reconciliation: no double marking

Power can die between "spray fired" and "COMPLETED recorded". The spray journal (`spray_controller`: `PRESS_COMMAND_SENT`, `PRESSED`, `RELEASE_UNCONFIRMED`, `COMPLETED`, `FAILED_SAFE`, `ABORTED_BY_STOP`, with `mission_run_id` + `point_id`) is the evidence.

| Journal for the resume point (same run id) | Marking tool | Decision |
|---|---|---|
| none / COMPLETED for an *earlier* point / FAILED_SAFE before press / ABORTED_BY_STOP before press | any | `NONE`: re-mark normally |
| `PRESS_COMMAND_SENT`, `PRESSED`, `RELEASE_UNCONFIRMED` | SPRAY | **`UNCERTAIN`: operator must choose** "Already marked → mark COMPLETED" or "Not marked → spray again". Resume is blocked until they choose |
| `COMPLETED` for this point but no point event | SPRAY | `CONFIRMED`: mark COMPLETED, resume from the next point |
| any | LASER | `NONE`: re-marking with the laser is harmless |

- The journal keeps its existing latch: a fault still needs `/spray/reset_fault`.
- The recovery decision is recorded in the runtime file and the report.

### C4. When the index is written

The backend writes `mission_runtime.json` from Mission Manager's status and point events.

**It writes only on these changes:**
- START (new run)
- state change (RUNNING / PAUSED / WAITING_FOR_NEXT / ERROR)
- a point event (ACTIVE, COMPLETED, SKIPPED, FAILED)
- a pause reason change
- a recovery decision
- a terminal event

**It writes in the same worker that checkpoints the live report**, `_report_checkpoint_worker`. That way the report and the index come from one snapshot and cannot disagree.

**Expected write rate:** a few writes per point.

`last_known` (raw fix and PX4 state) is refreshed at the same writes, and at most every 10 s while PAUSED. It feeds Part F.

---

## Part D: Restore operation in Mission Manager (item 4, accepted)

Mission Manager is the only authority for mission state. Today it has no way to accept progress: `_point_status` is only ever reset to `PENDING`.

### D1. New service `/mission_manager/restore_progress`

This is a new `RestoreProgress.srv` in `mission_manager_interfaces`, the ament_cmake package. It needs:
- `colcon build --packages-select mission_manager_interfaces`;
- a rebuild and restart of `mission_manager` and `rover_backend`.

```text
# request
string mission_id
string csv_sha256
string mission_run_id
string[] point_ids
string[] point_status        # COMPLETED | SKIPPED | FAILED | PENDING (ACTIVE sent as PENDING)
string resume_point_id
string execution_mode
string marking_tool
uint32 recovery_count
---
# response
bool success
string message
string state                 # PAUSED on success
```

### D2. Guards (all must pass, checked under Mission Manager's lock)

1. State is `READY`: a freshly prepared trajectory from the same CSV, and never started in this session.
2. The trajectory's marking point count equals `len(point_ids)`, and the prepared mission's id/hash match the request.
3. `resume_point_id` is the first unresolved point in `point_status`.
4. PX4 is **disarmed** (Part E3), E-stop is asserted (the backend boots with it asserted), and motion is disabled.
5. No spray transaction is open for this run, or C3 has been resolved.

### D3. Transition

On success:
- `READY → PAUSED` with `pause_reason = "RECOVERED"` and `resume_available = true`.
- The point statuses are seeded, the active point is set to the resume point, and `mission_run_id` stays the **same** so the report continues.
- A system event `MISSION_RECOVERED` is emitted.

Resume is then the **unchanged** normal RESUME. Its existing recovery path already covers a disarmed, non-OFFBOARD rover: zero-setpoint settle, then OFFBOARD, then ARM.

On failure: the state stays `READY`, and the backend shows "Recovery failed: <reason>". Only **Stop & Upload New Mission** is offered.

---

## Part E: Restart types and the recovery sequence (item 5)

### E1. What can restart, and what happens

| Event | How detected | PX4 | Mission Manager memory | Action |
|---|---|---|---|---|
| Full power cycle | new `boot_id` | rebooted, disarmed | lost | Recovery sequence (E2) |
| Jetson reboot only (PX4 stays powered) | new `boot_id` | **may still be armed** | lost | E3 first, then E2 |
| Stack restart (`systemctl restart dyx-rover`) | same `boot_id`, backend start | **may still be armed/OFFBOARD** | lost | E3 first, then E2 |
| Single node respawn (mission_manager) | Mission Manager status is EMPTY while the runtime says STARTED | may be armed | lost | E3 first, then E2 |
| Backend-only crash | backend start, Mission Manager still RUNNING/PAUSED | unchanged | **intact** | **No recovery.** Re-sync from Mission Manager and keep writing |

The last row is important: if Mission Manager still holds the mission, the backend must not restore anything. It only re-syncs.

### E2. Recovery sequence (the backend "recovery orchestrator", one worker thread)

```text
BOOT_WAIT      wait for ROS + mission_manager status + MAVROS connected (timeout 120 s → FAILED)
CHECK_FILES    load meta + runtime; stage == STARTED? else → NONE (no popup)
VERIFY_ID      B2 identity checks → else FAILED(reason)
CHECK_MM       mission_manager RUNNING/PAUSED/WAITING with same run id? → RESYNC (E1 last row), done
PX4_SAFE       E3
PREPARE        load active/mission.csv into the normal pipeline → trajectory_generator
               (same path as /api/mission/prepare; waits for the normal frame evidence)
WAIT_READY     mission_manager READY + trajectory ready (timeout 300 s → FAILED)
RECONCILE      C3 spray journal → mark UNCERTAIN if needed
RESTORE        call /mission_manager/restore_progress
OFFER          publish recovery block (Part G); popup on tablet
```

- It's shown to the tablet as `recovery.stage`, with a readable message for every step and failure.
- It runs **once** per backend start. Retrying is an explicit operator action ("Retry recovery").

### E3. PX4 safety before any restore

- If PX4 reports **armed**, the orchestrator does **not** disarm it silently. It holds at `PX4_SAFE` and shows "PX4 is armed: disarm to continue recovery".
- The motion gates still hold, because the backend boots with `emergency_stop = true` and `mission_enable = false`.
- ⚠ `NAV_DLL_ACT = 0` on this rover (CLAUDE.md), so a lost link does not disarm PX4. That is exactly why this step is explicit.

---

## Part F: The rover was moved while it was off (item 6)

The first leg after Resume starts from wherever the rover sits now. The production rule (task sheet P0) allows a runtime line from the current position **only for the C→P1 entry leg**. Every later leg tracks its fixed mission geometry.

**Plan:**
1. **Measure the displacement:** raw GNSS now vs `last_known` (C1). Compare only when both are fix_type 6.
2. **Show it in the popup:**
   - "Rover moved 1.8 m since power-off".
   - Above **0.30 m**, also say: "Place the rover near the line, or confirm".
3. **Decision needed (below):** treat the first leg after recovery as an *entry leg* (runtime line from the current position to the resume point, the same as C→P1), or require the rover to be near the saved position.
   - Recommendation: **entry-leg behaviour for the first recovered leg only**, because it's the same situation as starting a mission.
   - This needs one RPP flag set by Mission Manager on the first RESUME after `RECOVERED`.

---

## Part G: Operator interface and API

### G1. Mission status gains a `recovery` block (socket `mission_status` and REST)

```json
"recovery": {
  "available": true,
  "stage": "OFFER",
  "message": "Mission recovered: 6 of 9 done, resume from P0007",
  "saved_at": "…", "age_sec": 540,
  "mission_id": "…", "completed": 6, "skipped": 0, "failed": 0,
  "resume_point_id": "P0007",
  "uncertain_point_id": null,
  "displacement_m": 0.12,
  "blocked_reason": null
}
```

The tablet popup is driven **only** by this block. The tablet never computes recovery itself.

### G2. Actions

| Button | API | Effect |
|---|---|---|
| **Resume Mission** | existing `POST /api/mission/resume` | normal RESUME (needs E-stop released + RTK FIXED, as today; the popup says so) |
| **Stop & Upload New Mission** | existing `POST /api/mission/stop` + `POST /api/mission/clear` | stage → CLEARED, runtime archived to `records/`, active/ emptied |
| Resolve uncertain point | new `POST /api/mission/recovery/resolve-point` `{point_id, decision: "MARKED"\|"REMARK"}` | C3 |
| Retry recovery | new `POST /api/mission/recovery/retry` | re-runs E2 after a FAILED step |

`system_event` (from the prepared backend patch) also sends `MISSION_RECOVERED` / `RECOVERY_FAILED` for toasts.

---

## Part H: Settings storage, everything

### H1. What exists today (from code)

| Setting | Where now | Owner | Tablet-editable |
|---|---|---|---|
| Spray duration | `~/.ros/dyx_spray_settings.json` | spray_controller | ✅ |
| Default execution mode + marking tool | `~/.ros/dyx_mission_defaults.json` | mission_manager | ✅ |
| Obstacle enable/zones/port | `~/.ros/dyx_obstacle_settings.json` | obstacle_monitor | ✅ |
| Voice language/gender/on-off | `~/.ros/dyx_voice_settings.json` | rover_backend | ✅ |
| Laser PWM config | `/api/spray/laser-config`; persistence to verify | spray_controller | API only |
| RTK profiles, source, desired state | `rtk.sqlite3` | rover_backend | ✅ |
| Sessions/auth | `backend.sqlite3` | rover_backend | n/a |
| Rover identity, network, credentials, rates | `backend.env` | file | ❌ |
| **~325 tuning parameters** | hard-coded in `rover.launch.py` | git | ❌ |
| PX4 parameters | FCU flash (`param save`) | PX4 | ❌ (NSH/QGC) |
| Septentrio configuration | receiver NVRAM (configured once, by hand) | receiver | ❌ |
| Hotspot/QoS/systemd | NetworkManager, iptables, units | Jetson scripts | ❌ |

Gaps: every store has its own loader and writer, there are no schema versions, there's no audit trail, there's no backup or export, and there's no single view.

### H2. Target design

1. **Owner-local files, shared library.** Each setting file stays owned by the node that uses it, so nodes keep working without the backend. All of them move to `settings/` (A2) and use `dyx_persistence` (A3). Every file reports:
   - `source`: `saved` or `launch-default` (obstacle/spray already do this);
   - `revision`;
   - `updated_at` and `updated_by`.
2. **Settings registry in the backend: `GET /api/settings`.** One list of every domain, reading each owner's file or status. Per setting:
   - value;
   - default;
   - source;
   - range;
   - `apply`: `live` or `restart-required`;
   - `locked_while_running`.
3. **Change rules:**
   - Changes go through each domain's existing API (spray, obstacle, voice, RTK, mission defaults).
   - Settings that affect motion or marking are **refused while a mission is STARTED/RUNNING**, with a clear 409.
   - Every change appends a line to `records/audit/settings_audit.jsonl`: time, user, domain, key, old → new.
4. **Backup, export, import:**
   - Every settings change triggers an automatic snapshot, keeping the last 20 in `records/backups/`.
   - `GET /api/settings/export` downloads one versioned bundle.
   - `POST /api/settings/import`:
     - validates everything before writing anything, then writes all of it or none;
     - is refused during a mission;
     - is used to clone a rover or restore after a Jetson replacement.
   - The bundle excludes credentials, sessions, mission memory and journals.
5. **Tuning parameters (`rover.launch.py`): stay in git.** They are control-loop values and need review, so they stay off the tablet. If rovers ever need different values:
   - add an optional `settings/param_overrides.yaml` that the launch file reads;
   - it must use an **allow-list** with ranges;
   - it must appear in each run's as-run parameter capture.
6. **PX4 parameters:**
   - Keep a reference file per rover in the repo (`px4-params/<rover>.baseline.params`).
   - At boot, run a **read-only** comparison through the existing NSH bridge (`scripts/nsh_bridge.py param show`). Report any difference in health status and on the tablet ("3 PX4 params differ from baseline").
   - Add a snapshot command.
   - Changes still go through NSH/QGC + `param save`.
7. **Septentrio receiver:** keep it configured by hand (existing decision, no auto-config). Add a read-only snapshot of its configuration output into `records/` when it's changed.
8. **As-run capture:** each mission report records:
   - the settings bundle hash and revision;
   - the PX4 parameter difference against the baseline;
   - the launch-parameter hash.

   That way every run records exactly what was in effect.

---

## Part I: Implementation phases

| Phase | Content | Packages | Build impact |
|---|---|---|---|
| 1 | `dyx_persistence` library + tests; directory layout; migration of the 4 JSON settings files | new pkg, spray_controller, mission_manager, obstacle_monitor, rover_backend | `colcon build` once (new pkg) |
| 2 | Mission identity (B) + `mission_runtime.json` index writer (C1, C2, C4) in the report checkpoint worker | rover_backend | none (symlink) |
| 3 | `RestoreProgress.srv` + Mission Manager restore (D) | mission_manager_interfaces, mission_manager, rover_backend | **ament_cmake rebuild** |
| 4 | Recovery orchestrator (E) + spray journal reconcile (C3) + `recovery` block and APIs (G) | rover_backend, spray_controller (journal read API) | none |
| 5 | Tablet: recovery popup, uncertain-point dialog, toasts | DYX_GCS_Frontend | new tablet build |
| 6 | First recovered leg as entry leg (F), after the decision | mission_manager, rpp_controller | none |
| 7 | Settings registry, audit, backup/export/import (H2 1-4) | rover_backend + tablet | none |
| 8 | PX4 baseline comparison, receiver snapshot, as-run capture (H2 6-8) | rover_backend, scripts | none |

Phases 1-5 deliver the power-cycle recovery. Phases 7-8 can run in parallel.

### Tests

- **Unit (pure):** envelope/atomic write; a corrupt file becomes `.bad` + defaults; index resume-point rules; every C3 journal row; every B2 identity mismatch; every D2 guard; every E1 row.
- **Fault injection (bench, rover on stands):**
  1. `kill -9` of rover_backend at each E2 step.
  2. `kill -9` mid-write: the tmp file is left and ignored.
  3. Power cut while RUNNING.
  4. Power cut while PAUSED.
  5. Power cut while WAITING_FOR_NEXT.
  6. Power cut during an open spray press: expect UNCERTAIN.
  7. `systemctl restart dyx-rover` while armed: expect E3 hold.
  8. Replace mission.csv while off: expect identity refusal.
  9. Move the rover 2 m while off: expect the displacement warning.
- **Field:** a 9-point mission, cutting power at P7 in each marking mode; resume; check each point's mark is made **exactly once**.

---

## Part J: Decisions needed

1. **First leg after recovery:** entry-leg behaviour (recommended) or require placement near the saved position? (F3)
2. **Displacement threshold** for the warning: 0.30 m proposed.
3. **Recovery file age:** offer a mission of any age (showing the age), or expire after N days?
4. **Same `mission_run_id` after recovery** (recommended, so one continuous report), or a new run id linked to the old one?
5. **Tuning parameters:** confirm they stay git-only (recommended) versus an allow-listed override file.
6. **Settings import:** allowed on a rover with a different `DYX_ROVER_ID` (cloning), or same rover only?
