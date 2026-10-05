# Upgrade Extension — plan (4WD rover + DYX tablet)

**Status:** plan only, nothing implemented. Written 2026-10-04; operator decisions
recorded the same day (section 10).
**Repos:** `rover_ws` (rover: `rover_backend`, `trajectory_generator`, `mission_manager`)
and `DYX_GCS_Frontend` (tablet).

## 1. Goal

The operator can, from the tablet:

1. turn row extension **on or off** for the loaded mission;
2. **change its geometry** — the extension distance, the short-row trigger distance,
   and (optionally) skip individual row ends;
3. **see the result** on the map before Start: where each extension point is, and why a
   row end did or did not get one.

And it must be **clean**: a change or a disable leaves no trace anywhere — rover state,
stored mission, recovery checkpoint, report, or any tablet's map.

## 2. How it works today (verified from source, 2026-10-04)

- **Rover owns the geometry.** `trajectory_generator` adds one navigation-only point X,
  `D` metres past a row end Pn along the row direction, when all three hold
  (`topology.py::_evaluate_row_transition`):
  - gap Pn -> Pn+1 < `EXTENSION_TRIGGER_DISTANCE_M` = **3.0 m** (fixed);
  - sideways step: 45-135 deg from the incoming row;
  - next row runs back: >= 135 deg from the incoming row (needs Pn+2, so never at the end).
- `mission_manager` stops at X (30 mm, stationary), marks nothing there, then goes to Pn+1.
- **Settings travel only with the CSV upload** (`POST /api/mission/upload`:
  `extension_mode`, `dummy_point_distance_m`). Changing them means re-uploading the
  whole CSV, which creates a new `mission_id`.
- **The tablet never sees which points are extension points.** `trajectory_path`
  carries `x/y/lat/lon` only; status carries `dummy_point_count`.
- **Pending, uncommitted, not deployed:**
  - rover: `extension_distance_source` marker (unmarked uploads use 4.0 m, so old
    tablets' hard-coded 3.5 m stops leaking through);
  - tablet: `missionApi.ts` sends `extension_distance_source=OPERATOR`;
  - tablet: a "Row extension" switch in `MissionOpsPanel.tsx` / `PathPlanScreen.tsx`
    (uncommitted on `Feat/live-data-hardening`, author not confirmed).

## 3. Design principles

1. **Rover is the single source of truth.** The tablet never computes extension
   geometry; it edits settings and draws what the rover returns. (The 3WD moved geometry
   to its tablet and lost the server's safety checks on the way — do not repeat that.)
2. **One settings object, versioned.** Every change bumps a `revision`. Every derived
   artefact (trajectory, preview, recovery checkpoint, report entry) records the revision
   it was built from, and is discarded when the revision changes.
3. **Off means absent.** A disabled extension stores no distance, produces zero extension
   points, and yields a path identical (same `path_signature`) to a mission that never
   had extension enabled.
4. **Explicit apply.** No auto-apply while typing. Changes go through a dialog with OK.
5. **Refuse while moving.** Settings are editable only when no mission is running,
   paused, or waiting to be recovered.

## 4. Settings model

Stored in the mission metadata (`mission_store.py`), replacing the loose fields:

```json
"extension": {
  "schema": "dyx4wd/extension@1",
  "enabled": true,
  "distance_m": 4.0,
  "trigger_distance_m": 3.0,
  "skip_row_ends": [7],
  "source": "OPERATOR",
  "revision": 3
}
```

| field | editable | bounds (operator-decided 2026-10-04) | when disabled |
|---|---|---|---|
| `enabled` | yes | — | `false` |
| `distance_m` | yes | **1.0-20.0 m**, default 4.0, warn > 8 m | `null` |
| `trigger_distance_m` | yes | **1.0-5.0 m**, default 3.0 | `null` |
| `skip_row_ends` | yes (phase 3, scope to be agreed — section 6.4) | marking-point numbers that are row ends | `[]` |
| `source` | no | `OPERATOR` / `DEFAULT` | `null` |
| `revision` | no | +1 on every accepted change | kept (counts disables too) |

**Not editable** (safety, keep fixed): the 45-135 deg / >= 135 deg angle windows, the
direction rule (along the incoming row), minimum dummy clearance.

**Compatibility:** the legacy fields `extension_mode`, `dummy_point_distance_m`,
`dummy_point_distance_source`, `row_transition_threshold_m` stay written (derived from
`extension`) for one release so an old backend/tablet pair keeps working. Metadata without
an `extension` object is converted on load:
`enabled = extension_mode == ENABLE`, `distance_m` = 4.0 unless `source == OPERATOR`,
`trigger_distance_m` = 3.0, `revision` = 0.

## 5. Rover changes

### 5.1 Backend API (`rover_backend`)

| method | path | purpose |
|---|---|---|
| `GET` | `/api/mission/extension` | active mission's settings + bounds + latest decisions (5.3) |
| `PUT` | `/api/mission/extension` | replace settings; body includes `expected_revision` |
| `GET` | `/api/mission/extension/defaults` | saved defaults + bounds |
| `POST` | `/api/mission/extension/defaults` | save defaults (explicit operator action only) |

`PUT` rules:
- **409 `MISSION_ACTIVE`** when the mission is RUNNING, PAUSED, or a power-cycle recovery
  checkpoint exists for it. The operator must STOP/CLEAR first.
- **409 `REVISION_CONFLICT`** when `expected_revision` != stored revision (two tablets).
- **422** on out-of-bounds values (same bounds enforced on the tablet).
- On success, in this order: write metadata atomically (revision+1) -> emit
  `trajectory_path_cleared` (reason `extension_changed`) -> clear the cached live
  trajectory -> re-run PREPARE -> return the new settings.
- **Keeps `mission_id` and the CSV.** The mission identity is CSV + `extension.revision`.

Upload (`POST /api/mission/upload`) keeps its current fields and adds
`trigger_distance_m`; it writes `extension.revision = 1`.

Defaults live in `~/.ros/dyx_extension_defaults.json` (atomic write), used only to
pre-fill the tablet dialog and to fill an upload that omits values. Written only by
`POST .../defaults`.

### 5.2 Trajectory generator

- Read the `extension` object (fallback conversion above). Use `trigger_distance_m`
  instead of the class constant; honour `skip_row_ends`.
- **Identity fix (required for "no trace").** `_source_topology_is_current()` keys only on
  `mission_id` + CSV checksum, and its docstring says to add a compile-input signature if
  inputs can change without a new upload. With `PUT` they can. Add
  `extension.revision` to that identity.
- Disabled must produce exactly the DISABLE path; add a test asserting equal
  `path_signature` for "never enabled" vs "enabled then disabled".

### 5.3 Decisions and preview data

Publish, in `/trajectory_generator/status`, one entry per consecutive marking pair,
available at **COMPILED** (survey frame, no FCU needed) so the tablet can preview before
arming:

```json
"extension_decisions": [
  {"from": "P0007", "to": "P0008", "gap_m": 1.02, "transfer_deg": 90.4,
   "reversal_deg": 179.6, "decision": "ADDED",
   "point": {"lat": 13.0, "lon": 80.0}, "revision": 3},
  {"from": "P0003", "to": "P0004", "gap_m": 3.21, "decision": "SKIPPED_GAP"}
]
```

Decision codes: `ADDED`, `SKIPPED_GAP`, `DISABLED` (phase 2), and
`SKIPPED_OPERATOR` (phase 3). Only row ends are listed (one entry per row).

Dropped while building phase 2 (2026-10-05):
- `SKIPPED_ANGLE` (a short hop that is not a row end): the first hop into
  every new row is also a 90 deg turn from the hop before it, so real
  near-misses cannot be told apart and the list would be flooded.
- `SKIPPED_LAST_ROW`: the last pair has no following row and is simply
  not listed.
- The first pair of the mission is listed only when ADDED: the rule judges
  it against a synthetic incoming direction, so a long first row would
  read as a skipped row end.

`decisions` are served by `GET /api/mission/extension` only when their
revision equals the stored settings revision (`decisions_current`); mission
status carries `extension_decisions_revision` so tablets know when to refetch.

Extend `trajectory_path` (additive field, old tablets ignore it) with
`extension_indices` and `extension_revision`, so the placed path can be drawn with
extension legs coloured.

### 5.4 Clearance warnings (warn only — decided)

For each ADDED point, warn when X, or the straight leg X -> Pn+1, passes within
**0.30 m** of any other marking point. Reported as `warnings` on the decision and shown
on the tablet. **It never blocks Start** (operator decision 2026-10-04).

### 5.5 Mission manager, recovery, reports

- Motion behaviour unchanged.
- Recovery checkpoint records `extension.revision`; restore refuses (manual) if the
  stored mission's revision differs. With the 409 in 5.1 this should never trigger;
  it is the second lock.
- Mission report / point history records the settings and the decision summary used.
- Archive restore brings back the archived mission's own settings.

## 6. Tablet changes (`DYX_GCS_Frontend`)

### 6.1 Control

- Mission panel: **Row extension** row showing `OFF` or `ON · 4.0 m · trigger 3.0 m`.
  Tap -> **Row extension** dialog:
  - switch On/Off;
  - Distance (m), Trigger distance (m), each with the rover's bounds and a warning band;
  - (phase 3) row-end list from `extension_decisions` with a per-row-end skip toggle
    (see 6.4);
  - Cancel / **Apply**, and "Save as default".
- Pre-fill from the **rover** (`GET /api/mission/extension`, else defaults), never from
  local memory.
- Locked, with the reason shown, while a mission is running/paused/recovering or an
  upload/prepare is in flight.

### 6.2 Apply

- Mission already on the rover -> `PUT /api/mission/extension` with `expected_revision`.
  On 409 `REVISION_CONFLICT` reload and tell the operator another tablet changed it.
- No mission yet -> hold the settings in the upload form only, send them with the upload.
- **Remove** the current behaviour of re-uploading the whole CSV to change extension.

### 6.3 Preview

- Draw ADDED points and their legs in a distinct colour; show count and warnings.
- A "Why?" list of every row end with its decision in plain words
  ("P3->P4: no extension, gap 3.2 m is over the 3.0 m trigger").

### 6.4 Per-row-end control — for team discussion

Today the extension setting applies to the whole mission: every row end that meets the
rules gets the same distance. Per-row-end control lets the operator treat individual row
ends differently. The rover side is the same in every option: the rover still decides
which row ends qualify, and the operator can only change what happens at those.

```
Row 1:  P1 ──► P2 ──► P3 ─ ─ ─► X1      row end P3 (extension X1)
                               │
Row 2:  P6 ◄── P5 ◄── P4 ◄─────┘
        │
   X2 ◄─ ─ ─                            row end P6 (extension X2)
        │
Row 3:  └─────► P7 ──► P8 ──► P9
```

Options to choose from:

| option | what the operator can do | example | effort |
|---|---|---|---|
| A. Mission-wide only (today) | one on/off + one distance for all row ends | all X at 4.0 m | none |
| B. Skip a row end | turn off the extension at chosen row ends | no X2 because a fence is there | small: `skip_row_ends` list |
| C. Distance per row end | B, plus a different distance at chosen row ends | X1 at 4.0 m, X2 at 2.0 m (tight headland) | medium: `row_end_overrides` map `{ "P0006": 2.0 }` |

Rules that hold for B and C:
- Overrides are keyed by the row-end marking point ID (`P0006`) and belong to that mission
  and revision. A new CSV upload starts with none.
- An override on a point that no longer qualifies (e.g. after a trigger change) is dropped,
  listed in the response, and shown to the operator — never kept silently.
- The same bounds apply (1.0-20.0 m), and the same warnings.

Questions for the team:
1. Do we need per-row-end control in the first release, or is mission-wide enough?
2. If yes, is skip (B) enough, or do we need a different distance per row end (C)?
3. Where should it live on the tablet: a list in the dialog, or tapping the row end on the
   map?

### 6.5 No-trace rules on the tablet

- Clear all extension overlays on `trajectory_path_cleared`, on mission change, and on
  any revision change; redraw only from a payload whose `extension_revision` matches.
- Keep no extension geometry in local storage.

## 7. Acceptance tests ("no trace" checklist)

Rover (pytest):
1. Enable -> disable: metadata `distance_m == null`, 0 extension points, `path_signature`
   equals a never-enabled upload of the same CSV.
2. Enable(4.0) -> enable(6.0) -> enable(4.0): last signature equals the first.
3. `PUT` while RUNNING / PAUSED / recovery pending -> 409, nothing written.
4. Stale `expected_revision` -> 409, nothing written.
5. Topology cache is rebuilt after a `PUT` (same mission_id/checksum, new revision).
6. Old-format metadata (3.5 m, no source) loads as 4.0 m / trigger 3.0 / revision 0.
7. Recovery checkpoint with an old revision is refused.
8. Defaults file is written only by `POST .../defaults`.

Tablet (manual + unit):
9. Toggle off -> map shows no extension points on this and a second tablet.
10. Change on tablet A -> tablet B clears and redraws without a reload.
11. Old APK upload still works and gets 4.0 m.

## 8. Phases

| phase | content | repos |
|---|---|---|
| 0 | Land the pending work in section 2 (rover marker, tablet `OPERATOR` line, confirm owner of the uncommitted switch), deploy, field-check one serpentine at 4.0 m | both |
| 1 | Settings object + conversion, `GET/PUT` + revision + 409s, generator reads trigger, identity fix, tests 1-6 | rover |
| 2 | Decisions in status, `trajectory_path` extension indices, tablet dialog + Apply via `PUT` + preview + no-trace clearing, tests 9-11 | both |
| 3 | Per-row-end control (scope from 6.4), clearance warnings, defaults save/load, recovery/report revision, tests 7-8 | both |
| 4 | Field test: rows ~1 m apart, D = 2 / 4 / 6 m, trigger 3.0 and 2.0; measure X vs predicted position and time spent stopping at X | field |

## 9. Out of scope

- Relaxing the 30 mm stop at the extension point. Decide from phase 4 data
  (time spent creeping into X) in a separate change.
- Editable angle windows.
- Any tablet-side geometry computation.

## 10. Decisions

Decided by the operator, 2026-10-04:
1. **Extension distance minimum: 1.0 m** (today 0.1 m). Maximum stays 20.0 m, warning
   above 8 m.
2. **Trigger distance range: 1.0-5.0 m**, default 3.0 m.
3. **Clearance: warn only**, never blocks Start.

Still open:
4. Per-row-end control: scope and placement (section 6.4) — for team discussion.
5. Owner of the uncommitted "Row extension" switch on `Feat/live-data-hardening`.
