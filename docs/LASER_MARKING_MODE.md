# Marking tool (LASER / SPRAY) and default mission modes

Added 2026-09-28 on branch `latency-hardening`. It is unit-tested only and
**not yet field-tested**. The frontend buttons are not wired yet.

## Modes

| | Choices | Factory default |
|---|---|---|
| Execution mode | `MANUAL` (stop at each point, wait for NEXT) / `AUTO` | **MANUAL** |
| Marking tool | `LASER` (AUX6) / `SPRAY` (AUX5) | **LASER** |

- **Per mission.** Before START, set either with the endpoints below.
  Changes are refused while a mission is RUNNING, PAUSED or
  WAITING_FOR_NEXT.
- **Defaults.** `POST /api/mission/defaults` saves the *current*
  mode and tool to `/home/flash/.ros/dyx_mission_defaults.json`.
  - Every `mission_manager` start restores them.
  - With no file, the launch parameters `default_execution_mode` /
    `default_marking_tool` (MANUAL / LASER) apply.
  - `/api/mission/status` shows `default_execution_mode`,
    `default_marking_tool` and `defaults_source` (`launch` | `saved`).
- **LASER requires MANUAL.** START is refused with LASER + AUTO. Saving
  LASER + AUTO as the default is refused too. Nothing ever switches modes
  silently.

## LASER at each point

1. The rover stops at the point.
2. It passes the **same gates as spray**: RPP certificate and the raw-GNSS
   30 mm check. The laser then shows the result:

   | Raw-GNSS result at the stop | Laser until NEXT | Point |
   |---|---|---|
   | measured **≤ 30 mm** | **steady ON** — mark here | COMPLETED |
   | measured **> 30 mm** (`RAW_GNSS_RADIAL_EXCEEDS_TOLERANCE`) | **blinks 0.5 s ON / 0.5 s OFF**, repeating — the rover is here, do **not** mark | FAILED |
   | not measurable (RTK not FIXED, too few or scattered fixes) or no settled stop | OFF | FAILED |

   `spray_controller` drives the laser on **AUX6** (`MAV_CMD_DO_SET_ACTUATOR`
   param2; param1 NaN, so the spray servo is never touched).
3. PX4 ACKs ON → `spray_controller` reports SUCCESS (once) and enters
   `LASER_ON`.
4. `mission_manager` marks the point COMPLETED and enters `WAITING_FOR_NEXT`.
   It publishes `laser_hold = {mission_run_id, point_id}` in its status. The
   laser stays on with **no time limit**.
5. The operator presses **NEXT**. `mission_manager` clears `laser_hold`, then
   **waits for `/spray/status` to report `laser_on=false` after the request**
   (PX4-acknowledged OFF, up to `laser_off_confirm_timeout_sec` = 3 s).
   - Only then does it re-enable motion.
   - If OFF is not confirmed, NEXT is blocked with a message, the rover
     stays put, and NEXT can be pressed again.
6. **Final point:** it also waits for NEXT. NEXT turns the laser OFF, then
   completes the mission. This applies to a blinking final point too.

**Blink details:**
- `mission_manager` publishes `laser_hold.mode = BLINK`, and
  `spray_controller` starts a `LASER_BLINK` transaction from it (no marking
  request exists for a failed point).
- Each toggle is sent only after PX4 ACKs the previous one. The next toggle
  is due 0.5 s after the previous one was sent
  (`laser_blink_half_period_sec`).
- A rejected or timed-out toggle turns the laser OFF.
- No marking result is ever published for a blink.
- NEXT waits for OFF exactly as for a steady laser, because `laser_on`
  stays true for the whole blink.

**The laser is also turned OFF on any of these, with no fault latched:**
- SKIP;
- STOP;
- E-stop;
- mission complete;
- PX4 disarm, loss of OFFBOARD, or stale PX4 state;
- stale mission status;
- the spray controller disabled;
- the tool changed away from LASER;
- `laser_hold` not published within `laser_handoff_timeout_sec` (1.5 s) of
  SUCCESS.

OFF is retried until PX4 ACKs it, like a spray RELEASE.

**Failures:**
- If PX4 rejects laser ON, the point is reported FAILED, a fault latches
  (same rule as spray), and there is no hold.
- A controller restart that finds a laser transaction in its journal sends
  OFF and returns to IDLE without a fault.
- Shutdown sends spray RELEASE and laser OFF in one command.

## PX4 setup (do once per rover, in QGC)

| Parameter | Value | Why |
|---|---|---|
| `PWM_AUX_FUNC6` | `302` | Peripheral via Actuator Set 2 = param2 |
| `PWM_AUX_MIN6` / `PWM_AUX_MAX6` | `1000` / `2000` | must match the backend's laser PWM range |
| `PWM_AUX_DIS6` | `1000` (= OFF) | PX4 outputs this while disarmed, so the laser is off whenever disarmed |

Spray stays on AUX5 (`PWM_AUX_FUNC5=301`, param1).

## API

| Endpoint | Body |
|---|---|
| `POST /api/mission/marking-tool` | `{"marking_tool": "LASER"}` or `"SPRAY"` |
| `POST /api/mission/execution-mode` | `{"execution_mode": "MANUAL"}` or `"AUTO"` (existing) |
| `POST /api/mission/defaults` | — (saves the current mode + tool) |
| `POST /api/spray/laser-config` | `{"on_pwm_us": 2000, "off_pwm_us": 1000}` (1000–2000 µs, must differ; saved) |

**`GET /api/spray/config`** gains:
- `marking_tool`, `laser_on`, `laser_blinking`, `laser_on_point_id`;
- `laser_on_pwm_us`, `laser_off_pwm_us`, `laser_value_source`.

**The mission status** gains:
- `marking_tool`, `laser_hold_point_id`, `laser_hold_mode`
  (`STEADY` | `BLINK`), `laser_on`;
- `default_execution_mode`, `default_marking_tool`, `defaults_source`.

## Where it lives

**`spray_controller`:**
- `TOOL_LASER` and `STATE_LASER_ON`;
- `_laser_hold_failure()`;
- `_laser_config_callback()` (topic `/spray/laser_config`, volatile);
- laser values persisted in `dyx_spray_settings.json`;
- parameters `laser_actuator_set` 2, `laser_on_value` +1.0,
  `laser_off_value` −1.0, `laser_handoff_timeout_sec` 1.5.

**`mission_manager`:**
- `MARKING_TOOLS`, `_marking_tool`, `_laser_hold`;
- `_marking_tool_callback()` (topic `/mission_manager/marking_tool`,
  volatile);
- `_save_defaults_service()` (`/mission_manager/save_defaults`);
- `_wait_for_laser_off()`;
- completion and NEXT changes.

**Backend:**
- `ros_bridge.set_marking_tool`, `save_mission_defaults`,
  `set_laser_config`;
- `mission_routes.py` and `spray_routes.py`.

**Tests:**
- `spray_controller/test/test_laser_mode.py`
- `mission_manager/test/test_laser_marking_tool.py`
- `rover_backend/test/test_spray_routes.py`

## Known limits

- **Execution mode after a restart.** `/mission_manager/execution_mode` is
  still a *retained* topic (unchanged). If only `mission_manager` restarts
  while the backend keeps running, the backend's last execution-mode choice
  replays over the saved default. The marking tool topic is volatile, so it
  always starts from the default.
- **Frontend.** Tool selector, "save as default", laser PWM and the laser
  state are not built yet in `DYX_GCS_Frontend`.
