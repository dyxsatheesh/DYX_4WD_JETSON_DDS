# Spray manual control and hold-time API

Added 2026-09-28 on branch `latency-hardening`:

- `5f92816` — endpoints
- `3e87087` — duration endpoint
- `b16c158` — file persistence

These are unit-tested. They have **not been field-tested**.

All endpoints are under `/api/spray` on `http://192.168.3.101:5001` and
require login.

## Site workflow: tune the spray hold time

1. **Test.** `POST /api/spray/test` with `{"on": true, "duration_s": 0.30}`
   runs ON, holds 0.30 s, then OFF, once. Repeat with different times until
   the mark looks right.
2. **Save.** `POST /api/spray/duration` with `{"spray_duration_sec": 0.30}`.
   Every mission point after this uses 0.30 s. The value is saved on the
   Jetson and survives restarts and reboots.
3. **Check.** `GET /api/spray/config` returns `spray_duration_sec` and
   `spray_duration_source`: `"saved"` means the file is in use, `"launch"`
   means the `rover.launch.py` value.

Timing difference: the test hold is timed by the backend, starting when the
controller accepts ON. The mission hold is timed by the controller from
PX4's PRESS acknowledgement. The two differ by tens of ms, so confirm with
one real mark after saving.

## Endpoints

| Endpoint | Body | What it does |
|---|---|---|
| `POST /on` | — | Manual ON. The controller auto-releases after `manual_override_timeout_sec` (3.0 s); repeating ON does not extend it. |
| `POST /off` | — | RELEASE. Idempotent: always sends a RELEASE command. Cancels any test timer. |
| `POST /test` | `{"on": true, "duration_s": 0.2}` | Server-timed test. Default 0.20 s, capped at 3.0 s. `{"on": false}` cancels and releases. |
| `POST /enable` | — | Opens the runtime master gate (`enabled`). |
| `POST /disable` | — | Releases first, then closes the master gate. No spray happens at all, mission sprays included. Not persisted: resets to the launch value on restart. |
| `POST /duration` | `{"spray_duration_sec": 0.3}` | Saves the mission hold time. Must be > 0 and ≤ 3.0 s. |
| `POST /config` | `{"press_pwm_us", "release_pwm_us"}` | Existing endpoint. ON/OFF PWM (900–2000 µs), runtime only. |
| `GET /status`, `GET /config` | — | Controller state, hold time, manual and PX4 gate fields. |

## Responses

- **`success: true`** means the controller accepted the request. The field
  `physical_state_confirmed: false` means acceptance does not prove the servo
  moved. Watch `GET /status`.
- **`409`** means refused. The reason is in `detail`, for example:
  - `EMERGENCY_STOP_ACTIVE`
  - `PX4_STATE_UNSAFE`
  - `MISSION_ACTIVE:RUNNING`
  - `SPRAY_CONTROLLER_NOT_IDLE`
  - `SPRAY_SETTINGS_SAVE_FAILED`
- **`504`** means the outcome is unknown. `retry_safe` is `true` for
  OFF/enable/disable and `false` for ON.
- **`422`** means an invalid value.

## Manual spray safety contract (`spray_controller`)

**New ON is accepted only when all hold:**
- the controller is enabled, with no latched fault;
- it is idle;
- the E-stop is clear;
- mission status is fresh and the mission is not RUNNING, PAUSED or
  WAITING_FOR_NEXT;
- PX4 is fresh, connected and **armed** (OFFBOARD is not required);
- the MAVROS command service is ready.

**It releases at once on:**
- E-stop;
- PX4 disarm or disconnect;
- a mission starting;
- disable;
- OFF;
- the 3.0 s deadline.

`hard_press_timeout_sec` (5.0 s) remains an independent watchdog.

**Recovery and reporting:**
- Manual transactions are journalled like mission sprays, so a crash
  mid-spray means RELEASE plus a latched fault on restart.
- They never publish a point result or completion.

## Where it lives

**ROS services** (`std_srvs/SetBool`):

| Service | Purpose |
|---|---|
| `/spray/manual` | true = bounded PRESS, false = RELEASE |
| `/spray/set_enabled` | runtime master gate |

**Hold-time topic:** `/spray/duration_config` (String JSON, **volatile**).
Acknowledgement comes through the `config_request_id`, `config_result` and
`config_reason` fields in `/spray/status`.

It is volatile on purpose. A retained request that had been rejected must
not be replayed and applied after a later restart.

**Saved file:** `spray_settings_path` =
`/home/flash/.ros/dyx_spray_settings.json`. It is:
- written atomically before the value is applied (a save failure rejects
  the change);
- loaded at start over the launch value;
- ignored if missing, corrupt or out of range (logged).

Delete the file to return to `rover.launch.py`'s `spray_duration_sec`.

**Backend:**
- `rover_backend/spray_routes.py`
- `ros_bridge.py`: `set_manual_spray`, `set_spray_enabled`,
  `set_spray_duration`

**Tests:**
- `src/spray_controller/test/test_manual_spray.py`
- `src/rover_backend/test/test_spray_routes.py`

## Not done yet

- **Frontend.** `DYX_GCS_Frontend` has no Test button wired up, and its
  Save sends only PWM, not `spray_duration_sec`.
- **Known limits:**
  - `/off` can queue briefly behind an in-flight `/on`, because of the route
    lock.
  - PWM ON/OFF values are runtime only, not persisted.
