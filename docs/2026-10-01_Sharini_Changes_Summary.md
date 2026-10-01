# What SHARINI changed (2026-09-10 -> 2026-10-01)

Written 2026-10-01 from the commits and the **current source**. SHARINI commits
under two git identities: `SHARINI` and `ShariniSenthil` (same person; search
with `git log -i --author=sharini`).

**How each statement was checked**

- **[source]** read in today's source tree (not just the commit diff).
- **[diff]** read from the commit diff only; a later commit may have changed it.
- **[bag]** decoded from the 2026-09-30 bag `mission.csv_20260930_185447`.
- **Field-tested?** Nothing here was field-verified by me. Where the operator
  reported a result, it is not re-checked against logs.

---

## 1. Battery: percentage, critical pause, lockout (2026-09-30 -> 10-01)

| Commit | Date | Change |
|---|---|---|
| `9268a87` | 09-30 10:59 | Lock mission operation at critical battery: `battery_policy.py`, 409 lockout in `mission_routes.py`, new `battery` state fields |
| `1e587d9` | 09-30 11:39 | Battery percentage is **derived from pack voltage**, smooth and whole-number |
| `1097940` | 09-30 11:51 | Critical voltage raised to 23.80 V |
| `ef14c9a` | 09-30 16:01 | Critical now **PAUSES** a RUNNING mission after a debounce; the bridge requests battery data at 2 Hz |
| `5602d83` | 10-01 10:52 | **Critical = 22.00 V held for 20 s** (was 23.80 V / 5 s) |

**Current behaviour [source: `battery_policy.py`, `ros_bridge.py`]**

- **Percentage** comes from voltage, not PX4's own number: 26.50 V = 100%,
  22.00 V = 10%, linear in between, clamped to 11..99 strictly inside the range.
  At or below 22.00 V the raw value is 10%.
- **Debounce.** `BatteryCriticalDebounce` confirms critical only after
  **22.00 V is held for 20 s** with at least 2 low samples, and the last low
  sample no older than **1.5 s**. One sample above 22.00 V cancels both a
  pending and a confirmed critical. Missing or stale telemetry can never create
  a critical lock.
- **While pending** the tablet sees **11% / LOW**, not 10% / CRITICAL.
- **When confirmed:** `battery.mission_lockout = true` (`BATTERY_CRITICAL`),
  status CRITICAL, voice announcement, and, only if the mission is RUNNING, a
  **PAUSE** through Mission Manager's `pause_battery` service
  (`pause_reason = BATTERY_CRITICAL`). It does not stop or clear the mission.
- **Lockout.** START, RESUME, NEXT and SKIP return **HTTP 409
  `BATTERY_CRITICAL_LOCKOUT`** (`_require_battery_mission_operable`). START
  re-checks right before dispatching to ROS.
- **Data rate.** `cmd_vel_bridge` asks PX4 for MAVLink `BATTERY_STATUS` (147)
  at `battery_status_rate_hz` (launch sets 2.0; node default 0.0 = off).
  Without it `/mavros/battery` runs at about 0.3 Hz and the 1.5 s freshness
  rule would never be satisfied.

**Known rough edges [source]**

- Docstrings and a log line in `battery_policy.py` / `ros_bridge.py` still say
  "23.80 V" and "five seconds". The constants are authoritative:
  `BATTERY_CRITICAL_VOLTAGE_V`, `BATTERY_CRITICAL_DEBOUNCE_SEC`.
- No unlock hysteresis: a single sample above 22.00 V clears the lockout, and
  the pack recovers at rest.
- A 22.00 V threshold on an 8S LiFePO4 pack (3.00 V/cell empty per our
  `BAT1_V_EMPTY`) sits at 2.75 V/cell, **below** that empty voltage. Whether
  that is the intended margin is an operator decision, not checked here.
- Deployment of `5602d83` on the Jetson was not checked.

---

## 2. Power-cycle mission recovery (`b6eb272`, 2026-09-30 15:13)

**Goal:** if the Jetson loses power mid-mission, offer the operator to resume
the unfinished mission after boot, without ever moving the rover on its own.

**Flow [source: `main.py`, `mission_recovery.py`, `recovery_runtime.py`,
`mission_manager_node.py`]**

1. **Checkpoint.** While a mission is RUNNING / PAUSED / WAITING_FOR_NEXT with a
   `mission_run_id`, the backend writes `mission_recovery.json` (next to
   `mission_runtime.json` in the runtime dir). It is deleted on EMPTY /
   COMPLETED. My own later change `d2828a0` merges the per-point evidence
   from `/mission_manager/point_history` back into it.
2. **Validate** (`validate_power_cycle_recovery_payload`): schema version 1,
   same `mission_id` **and** same CSV checksum as the stored mission, state in
   the recoverable set, a `mission_run_id`, `point_status` of the right length,
   and evidence arrays either absent or exactly the point count. Anything
   else means no recovery.
3. **Backend state machine** (`_run_power_cycle_recovery`), exposed to the
   tablet as `recovery_state`: `CHECKING` -> `PX4_SAFE` (waits for MAVROS
   connected **and PX4 disarmed**; it never disarms for you) -> `PREPARING`
   (rebuilds the trajectory) -> `RESTORING` (calls
   `/mission_manager/restore_runtime`) -> `READY`, or `ERROR`; `RESYNCED` when
   there is nothing to recover.
4. **Restore plan** (`build_recovery_plan`): terminal points (COMPLETED /
   SKIPPED / FAILED) and their results are kept. **Any in-progress point goes
   back to PENDING.** The first unresolved point is the resume target.
   Evidence for unresolved points is cleared.
5. **Never restored:** timers, spray / laser state, RPP certificates, OFFBOARD,
   arming, auto-continue. Mission Manager comes up **PAUSED** with
   `pause_reason = POWER_CYCLE_RECOVERY` and `resume_available = true`.
6. **Resume** is the normal Resume. After a boot the backend asserts the
   E-stop, so Mission Manager withholds `resume_available` until it is
   released. The tablet change `b628853` makes the popup do **Release E-stop &
   Resume**.

New interface: `RestoreMissionRuntime.srv` (`mission_manager_interfaces`,
needs `colcon build`). Tests: `test_mission_recovery.py`,
`test_recovery_runtime.py`.

Important: `trajectory_generator`'s `path_signature` hashes local x/y and
changes every reboot, so mission identity across a reboot is the CSV SHA-256.

---

## 3. RPP / yaw transport (2026-09-11 -> 09-21)

### Explicit yaw and yaw-rate to PX4 (**contradicts older sections of CLAUDE.md**)

| Commit | Date | Change |
|---|---|---|
| `9d98d6f` | 09-11 | `RPP_EXPLICIT_YAW_ENABLED = True` in `rover.launch.py` |
| `102d103` | 09-18 | **"Jetson full-authority yaw-rate commands"**: `RppCommand.msg` gains `yaw_rate_enu_radps`; `cmd_vel_bridge` forwards yaw **and** yaw-rate |

- **[source]** `cmd_vel_bridge.py` defines `TYPE_MASK_VELOCITY_ONLY` (0xdc7) and
  `TYPE_MASK_VELOCITY_YAW_RATE` (0x1c7, `IGNORE_YAW` and `IGNORE_YAW_RATE`
  cleared). Yaw-rate is clamped to `maximum_yaw_rate_radps` (bridge default
  0.45).
- **[bag]** In the 2026-09-30 bag, **2006 of the first 4000 setpoints use
  `type_mask 0x1c7`** with yaw in -1.72..+1.74 rad and yaw-rate up to
  0.75 rad/s; the other 1994 use `0xdc7` with zero yaw.
- **Flag semantics [diff]:** `true` = both yaw and yaw-rate are authoritative;
  `false` = no yaw authority, and downstream must fail closed to a no-rotation
  safety stop rather than silently falling back to vector steering. It is
  restart-only.
- So the statements in CLAUDE.md that "no yaw is commanded at all" and "PX4
  derives heading from the velocity vector" describe **2026-09-01..09-04** and
  are **not true of the current build**. The 1.0 s heading-lag analysis there
  was measured on the velocity-only contract.

### Pivot, moving steering, straight tracking

| Commit | Date | What [diff] | Still current? |
|---|---|---|---|
| `d1d982a` | 09-18 | Pivot max yaw-rate 0.20 -> 0.35, min 0.06 -> 0.10, kp 1.0 -> 1.5; new moving-yaw params; `xtrack_priority_speed` 0.20; radial-stop `minimum_actuatable_speed` 0.15 and `stop_lead` 0.035 m; `terminal_stop_regulator.py` +39 lines | `xtrack_priority_speed` reverted next commit; the two `radial_stop_minimum_*` keys are **not in today's `rover.launch.py`** |
| `c39578d` | 09-18 | Keeps the strong stationary pivot, restores moving-yaw to the `102d103` values, `xtrack_priority_speed` back to cruise | partly (later retuned) |
| `084b7b4` | 09-19 | "Save current Jetson tested configuration": cruise 1.0 -> 0.6, acceleration distance 1.0 -> 0.2, deceleration off, `pivot_enter_angle_deg` 45 -> 15, terminal native pivot enter 45 -> 15. **Also committed two rosbag test folders (about 59 MB of `.db3`) into git.** | Superseded: cruise is 1.0 today |
| `d38b87b` | 09-19 | "pivot 4 degree": `moving_yaw_kp` 1.0 -> 0.85; stationary pivot follows the interpolated `/nav_path` tangent (`path_bearing`) instead of the fixed semantic segment bearing | not re-checked |
| `cbad059` | 09-21 | **`precision_guidance_enabled` True -> False**: production straight legs use `line_guidance()` on the fixed path tangent. Adds moving-yaw deadband, rate slew and `moving_alignment_min_speed` | **Yes, `False` today [source]** |
| `d5c9bd0` | 09-21 | "Straight tracking v1": deceleration on, 0.20 m; yaw deadband 0.20 / 0.40 deg | deadband later tightened to 0.15 / 0.30 (Vetrivelan, 09-23) |

**Current values in `rover.launch.py` [source, 2026-10-01]:** pivot enter 15 deg
/ exit 6 deg, pivot max yaw-rate 0.75, min 0.06, pivot kp 1.80, moving yaw-rate
max 0.18, moving kp 0.85, moving deadband 0.15 / 0.30 deg, moving slew 0.60
rad/s^2, `precision_guidance_enabled False`, `geometry_tracking_enabled True`,
`xtrack_priority_speed_mps = CRUISE_SPEED_MPS`. The pivot-angle and
yaw-rate values come from later commits by others (see `git log -S`).

Consequence for CLAUDE.md: the 2026-09-04 "lookahead that actually governs is
time-based" section assumed `precision_guidance_enabled: true`. With it
`False`, straight legs steer with `line_guidance()` and its own lookahead, so
that analysis **does not describe the current build**.

---

## 4. Trajectory and mission manager (2026-09-11)

- **`61a3aa7` "4 m row-end extension logic" [diff]:** `trajectory_generator`
  (+124/-33 lines) and a `config.py` default. Extension is a **row-end
  manoeuvre, not a generic short-segment rule**, active only when
  `extension_mode == "ENABLE"`; it uses the incoming row heading and the
  transfer / reversal angles (`_extension_transition_geometry`). The
  2026-09-30 trajectory log said `Extension mode : DISABLE`, so it was off in
  that run.
- **`ce25a4a` "keep RTK float warning-only during runtime" [diff]:** was the
  runtime RTK policy on 09-11. **Superseded** on 09-26 by the pause + auto-resume
  policy (CLAUDE.md, RTK section).

---

## 5. Other

- `c3f53d0` (09-10), `ce8a0e5` (09-18): a test stub and docstring escaping
  fixes. Nothing behavioural.

## Where this leaves tomorrow

- Battery 22 V / 20 s and recovery are the newest, least-exercised changes. The
  recovery flow was exercised on the rover on 2026-09-30 (operator report;
  the popup case is covered by `b628853`), not re-verified here from logs.
- If anything yaw-related behaves oddly, remember the build now sends explicit
  yaw and yaw-rate, so PX4 is no longer inferring heading from velocity alone.
