# DYX 4WD Marking Rover — Jetson Companion

Scope: ROS2 Humble colcon workspace on Jetson `192.168.3.101`, PX4 OFFBOARD
control of a 4-wheel skid-steer marking rover. Modeled on the sibling 3WD
project's CLAUDE.md (`PX4_DXP/CLAUDE.md`) — **do not assume any fact from
that file applies here.** The two rovers share almost no infrastructure:
different repo, different Jetson, different FCU, different motor
controllers, different subnet. Every fact below was verified against this
repo's own source/params/bags on 2026-08-31 (agent research + direct bag
decoding); citations are file:line where available. **A full field session on
2026-09-01 added §"2026-09-01 field session" below and corrected several
facts — read that section before trusting anything speed- or pivot-related
elsewhere in this file.**

> **This file is new (2026-08-31).** It has survived exactly one field
> session's worth of corrections (2026-09-01), not the many that
> `PX4_DXP/CLAUDE.md` has — treat anything that later turns out wrong as an
> invitation to fix this file, not as a reason to distrust the whole thing.

## PX4 NSH through SSH — verified 2026-09-07

**Read [the NSH/SSH runbook](docs/PX4_NSH_OVER_SSH.md) before using PX4
`param`, `listener`, or `logger` remotely, or repeating the ground symmetry
test.** It contains the exact transport code, successful test harness,
execution result, and observed failure cases. This method does not require
a stack restart or another connection to the FCU serial device.

- SSH reaches **Jetson Linux**, not PX4 NSH. The verified route is
  `flash@192.168.3.101` → ROS2 Humble → publish
  `mavros_msgs/msg/Mavlink` on `/uas1/mavlink_sink` → existing MAVROS serial
  connection → MAVLink `SERIAL_CONTROL` → PX4 NSH. Replies arrive on
  `/uas1/mavlink_source`. Do not open `/dev/ttyACM0` alongside MAVROS.
- Exact shell settings: MAVLink 2, message ID `126`, shell device `10`,
  RESPOND flag `2`, timeout/baudrate fields `0/0`, encoder
  `srcSystem=1, srcComponent=191`. Newline-terminate commands, split at
  70 bytes and zero-pad; use pymavlink plus `mavros.mavlink` conversion
  helpers. Successful raw receive QoS: best effort / volatile, depth 1000.
- Source `/opt/ros/humble/setup.bash`; temporary dependencies were installed
  with `python3 -m pip install --target /tmp/ground_symmetry_deps pymavlink`.
  Set `MAVLINK20=1` before imports, then add that directory with
  `sys.path.insert`; do not overwrite the ROS `PYTHONPATH`.
- Foreground test SSH options were
  `ssh -tt -o BatchMode=yes -o ServerAliveInterval=2 -o ServerAliveCountMax=3`,
  with a local PTY and `python3 -u`. Wait for the echoed NSH command and a
  subsequent `nsh>`; send shell `\x03` to stop a listener, then an empty
  SERIAL_CONTROL message with flags 0 when closing the shell.
- NSH `ver all` confirmed PX4 1.16.2 at
  `54f0455ffcd755534539a7cf33a09a20bf71d29d` on 2026-09-07. ⚠ **Since a
  2026-09-25 reflash the FCU runs a custom build**: git
  `1fe17890895d4682db1f2ec5f3203c789c731831`, built Sep 25 2026, variant
  `differential`, still reporting 1.16.2 (NSH `ver all`, 2026-09-29). Use
  **that** commit for receiver/mixer/driver inspection. For routine reads,
  `scripts/nsh_bridge.py "<cmd>" ...` on the Jetson wraps this transport
  (refuses to run while armed unless `--allow-armed`).
- For the **authorized ground test only**, temporarily set/save
  `COM_RC_IN_MODE=1` while **disarmed**, then start
  `/mavros/manual_control/send` (`mavros_msgs/msg/ManualControl`) at 50 Hz
  with `x=0, y=0, z=500, r=0`. Steering is **y**, in raw MAVLink units:
  LEFT = -400/-550/-700, RIGHT = +400/+550/+700. **z=0 is full reverse**.
  `/mavros/actuator_control` is unsupported by this PX4 receiver; the existing
  `scripts/acro_yaw_manual_control_test.py` has the wrong mapping for this test.
- Verify fresh, valid PX4 `manual_control_setpoint` with
  `listener manual_control_setpoint -n 100000 -r 5` (5 Hz observation,
  separate from 50 Hz publication). Then enter MANUAL **while disarmed**,
  confirm 3 seconds stable with neutral/no reported failsafe, have the operator
  arm manually, and hold neutral another 3 seconds before steering.
  Never recover an armed HOLD attempt by forcing MANUAL and continuing.
- Keep neutral publishing on completion/abort/Ctrl-C, disarm, check the RC
  arm switch is DISARM, then `param set COM_RC_IN_MODE 0` → `param save` →
  `param show COM_RC_IN_MODE`; verify saved 0 and disarmed again. An earlier
  attempt **rearmed from the RC switch when input authority was restored**.
- E-stop release uses authenticated `POST /api/estop/release` and the manager's
  generation guard. The operator's live DYX app had **no RELEASE button**,
  despite one existing in local frontend source. Do not assume deployed UI
  parity. Do not publish `/emergency_stop=false` or clear a new stop automatically.
- The complete successful ULog is
  `/fs/microsd/log/2026-09-07/07_34_22.ulg`. No failsafe occurred during its
  timed sequence; PX4 printed “Preflight Fail: No CPU and RAM load information”
  **after disarming**. That message remains unexplained. Earlier FTP/parameter
  pull timeouts were observed, but their causal role in failsafe was not proven.
  Use NSH and logger announcements during the test; do bulk reads beforehand.

## Hardware

| Item | Value | Source |
|---|---|---|
| Jetson IP | `192.168.3.101` (user `flash`) — since 2026-09-28 this is the Jetson's **own Wi-Fi hotspot** `DYX-4WD` (no router); a cable in `enP8p1s0` adds a DHCP address (`192.168.1.37` on the bench) | `src/rover_backend/config/backend.env`, `docs/JETSON_HOTSPOT_NETWORK.md` |
| Wi-Fi | built-in **RTL8822CE**, vendor driver `rtl88x2ce`, AP on 2.4 GHz ch 6, txpower 15 dBm | `docs/JETSON_HOTSPOT_NETWORK.md` |
| Internet uplink | USB 4G dongle (`usb3`, `192.168.8.x`) — carries NTRIP; shared to hotspot clients, capped | `docs/JETSON_HOTSPOT_NETWORK.md` |
| GCS UDP target | `192.168.3.105:14550` — ⚠ nothing holds `.105` on the hotspot (ARP INCOMPLETE); reserve it for the QGC laptop if QGC is used | `src/rover_bringup/launch/rover.launch.py:102` |
| OS | ROS2 Humble (EOL May 2027) | `REFACTOR_TO_CPP_AUDIT.md` |
| FCU | **Holybro Pixhawk 6X** — NOT a CubeOrangePlus | `src/spray_controller/spray_controller/spray_controller_node.py:6`, `rover.launch.py:174` |
| FCU connection | `/dev/ttyACM0` @ 921600, `tgt_system=1 tgt_component=1 fcu_protocol=v2.0` | `rover.launch.py:47-59` |
| Drive | 4-wheel skid-steer **through a RoboClaw** (operator-confirmed 2026-09-29). PX4 `roboclaw` driver on TELEM3 = `/dev/ttyS1`: `RBCLW_SER_CFG=103`, `RBCLW_BAUD=115200`, `RBCLW_ADDRESS=128`, `RBCLW_FUNC1/2=101/102` (Motor 1/2), `RBCLW_VEL_CTRL=1` (encoder closed-loop), `RBCLW_COUNTS_REV=160000`, `RBCLW_QPPS_MAX=171000`, `RBCLW_ENC_HZ=20`. Airframe `SYS_AUTOSTART=50000` (generic differential rover). `PWM_MAIN_FUNC1..4` were still 101/102/101/102 (old Sabertooth PWM path driven in parallel); **set to 0 and saved 2026-09-29**, so RoboClaw is the only motor output. | NSH `param show`/`set`, 2026-09-29 |
| Drive (history) | Until some point before 2026-09-29 this rover drove **dual Sabertooth 2X32** via `PWM_MAIN_FUNC1..4` with RoboClaw disabled (`RBCLW_SER_CFG=0`). Every speed/throttle/pivot number measured on the Sabertooth drivetrain (e.g. §2026-09-01 `RO_SPEED_*` droop, throttle->speed slope, breakaway) **may not hold on RoboClaw** — re-measure before relying on it. The switch date is not recorded. | `px4-params/*.params` (stale) |
| Battery | **8S LiFePO4, 30 Ah** (operator, 2026-09-29). `BAT1_N_CELLS=8`, `BAT1_V_CHARGED=3.45`, `BAT1_V_EMPTY=3.00`, `BAT1_CAPACITY=30000`, set via NSH 2026-09-29 (3.45/3.00 show no `+` in `param show` because they equal this build's defaults — blank means default, not unsaved). Before that `N_CELLS=3`/`V_CHARGED=4.70` made a 26 V pack read 8.66 V/cell -> "Battery 1: over voltage" blocked arming. The INA228 current channel reads ~0.1 A and does **not** measure traction current, so PX4 SoC is voltage-only (LFP curve is flat — treat % as rough). | NSH, `commander check` OK after |
| GNSS receiver | **Septentrio mosaic-H**, dual-antenna, SBF protocol, 10 Hz PVT, dedicated PX4 driver `src/drivers/gnss/septentrio/` — **not** a u-blox F9P; audit the Septentrio path, not UBX. **Firmware 4.14.0 (2023-05-22)**, serial 3804732. It does not use BDS in RTK; cause **not yet established** (see §"Base corrections vs receiver use": the base sends BDS almost entirely single-frequency). A firmware upgrade is reasonable maintenance but is **not** an evidence-backed fix | `docs/2026-09-04_EKF_GNSS_Hardware_Audit.md` §8; `lstInternalFile, Identification` 2026-09-30 |
| MAVROS | apt 2.14.0 **plus a patched overlay** at `~/mavros_ws` (`scripts/build_patched_mavros.sh`: upstream duplicate-ACK / param race fixes). The launch log line `MAVROS: patched overlay` confirms it is in use | 2026-09-30 |
| Wheel track | `RD_WHEEL_TRACK = 0.63 m` | `px4-params/*.params` |
| | Full NTRIP client (`rtk_correction_bridge`), managed as a supervised child process by `rover_backend`, confirmed FIXED and live 2026-08-31 evening | see RTK section below |

## Control architecture — verified against live source, NOT the stale status doc

```
rpp_controller  →  /rpp/velocity_ned (Vector3Stamped, m/s, North/East)
                →  cmd_vel_bridge (jetson_4wd_control pkg)
                →  /mavros/setpoint_raw/local (PositionTarget, FRAME_LOCAL_NED, velocity-only mask)
                →  PX4 OFFBOARD
                →  RoboClaw (PX4 roboclaw driver, TELEM3) — was Sabertooth 2x32 via PWM before
```
(`cmd_vel_bridge.py:127-132, 158-162`)

Since 2026-09-01 the line `rpp_controller` tracks is not always `/nav_path`:
after any pivot certifies, it rebuilds a straight runtime line from where the
pivot actually left the rover to the current segment goal, and follows that
for the rest of the leg (`post_pivot_reanchor_all_legs`, default true). The
goal is never moved and `/nav_path` is never modified — see the 2026-09-01
session section for why this is safe and what it fixed.

**`WORKING_STATUS_24_06_2026.txt`'s architecture is stale** — it describes a
`corner_controller` node that no longer exists anywhere in `src/`
(superseded by `rpp_controller`) and a `/cmd_vel`→`/mavros/setpoint_velocity/cmd_vel`
path that isn't what's wired today. **`REFACTOR_TO_CPP_AUDIT.md`'s own ASCII
diagram is also simplified/inaccurate** on this point — trust the source
above over either document.

Key behaviors of `cmd_vel_bridge` (`src/jetson_4wd_control/jetson_4wd_control/cmd_vel_bridge.py`):
- Never stops streaming — publishes zero or the live command. This is how the
  "≥2 Hz before OFFBOARD" rule below is satisfied structurally, not by
  discipline. **Since 2026-09-25** (not yet field-verified) each RPP command
  is forwarded **on arrival**, and RPP runs at **50 Hz** (`control_rate_hz`
  in `rover.launch.py`; node default 20). The 50 Hz timer re-evaluates every
  safety gate, publishes at once when the gate outcome changes, and repeats
  the last command only as a keep-alive after 1.25 periods of silence.
  Before this, a 20 Hz RPP was resampled by the 50 Hz timer, so each command
  went out 1–3 times (measured 25_09 stage_9). The bridge also asks PX4 for
  `LOCAL_POSITION_NED` at 50 Hz (`local_position_rate_hz`) on every MAVROS
  connection; the PX4 default on this link measured 28–30 Hz.
- **No yaw is commanded at all.** Confirmed 2026-09-01 by decoding every
  `/mavros/setpoint_raw/local` message in seven bags: `type_mask = 3527
  (0xdc7)` sets **both `IGNORE_YAW` and `IGNORE_YAW_RATE`**, and the `yaw` /
  `yaw_rate` fields are identically zero. The contract is VX/VY only, and PX4
  derives heading from the velocity vector. Don't go looking for a yaw-rate
  command topic — there isn't one, and adding one would not help: PX4
  velocity OFFBOARD discards `trajectory_setpoint.yawspeed`.
  Pivoting is handled entirely by PX4's own differential-rover logic
  (`RD_TRANS_DRV_TRN` = 45°, matching `rpp_controller`'s
  `pivot_enter_angle_deg: 45.0`). ⚠ **The exit angles no longer match**:
  `RD_TRANS_TRN_DRV` is `0.10472 rad = 6°` on the live FCU while
  `pivot_exit_angle_deg` is still `12.0` in `rpp_controller.yaml`. Older
  `px4-params/` exports say 12° — they are stale on this param.
- MAVROS fields are ENU (`x`=East, `y`=North), converted explicitly at
  lines 297-300. Confirmed independently by decoding
  `/mavros/local_position/odom` in a real bag (§2 of the `analyse-missions`
  skill) — same convention, cross-checked two ways.
- Hard ceiling `ABSOLUTE_MAXIMUM_SPEED_MPS = 1.00` (line 47), **not**
  parameter-overridable (lines 86-89).
- Non-zero velocity requires, in order: `emergency_stop` clear →
  `mission_enabled` → backend heartbeat healthy → `mavros/state.connected` →
  `armed` → `mode == "OFFBOARD"` → RPP command fresher than
  `command_timeout_sec` (0.25s). Any failure → literal zero, never a
  stale/cached command (`_control_loop`, lines 313-343).

Arm/OFFBOARD sequencing is owned by `mission_manager`, not `cmd_vel_bridge`
(`mission_manager_node.py:13`): stream settle 0.60s → request OFFBOARD →
confirm OFFBOARD **while still disarmed** → settle 0.50s → confirm OFFBOARD
again → then ARM (`mission_manager_node.py:1853-1879`). A background monitor
(lines 1414-1429) stops motion if PX4 unexpectedly leaves OFFBOARD or
disarms, but explicitly does **not** auto-recover ("NEVER forces PX4 back to
OFFBOARD and NEVER requests" — line 1416-1417 docstring); that's an operator
action.

## ⚠ `scripts/start_rover.sh` is BROKEN — do not tell anyone to run it

It launches `ros2 run offboard_controller cmd_vel_bridge` — package
**`offboard_controller` does not exist** anywhere in `src/` (only
`jetson_4wd_control` provides a `cmd_vel_bridge` executable, per
`src/jetson_4wd_control/setup.py:21`). It also uses the wrong FCU baud
(115200 vs the production launch's 921600) and never starts `rover_backend`
or `spray_controller`. Git history shows it was only ever touched in the
initial commit and never updated as the architecture moved on.

**The authoritative way to bring the stack up is
`src/rover_bringup/launch/rover.launch.py`**, not `start_rover.sh`.
**Since 2026-09-29 it can run as the boot service `dyx-rover.service`**
(`scripts/rover_launch.sh`, installed by the operator with
`sudo ./scripts/install_rover_service.sh --now`; runbook
`docs/ROVER_BOOT_SERVICE.md`). **It is installed and in daily use (verified
2026-09-30)**: "restart the stack" is `sudo systemctl restart dyx-rover`, and
nobody should also launch by hand. The bag recorder is a separate unit,
`bag-autorecord.service`; restart it too after changing
`scripts/bag_autorecord.py`.

`scripts/stop_rover.sh`'s disarm sequence is still broadly sound
(disable mission → e-stop → zero velocity → disarm → MANUAL) even though it
publishes to a topic (`/mavros/setpoint_velocity/cmd_vel`) the current
architecture doesn't read — the disarm/mode-change service calls it makes
are independent of which velocity topic is wired up, so the sequence still
achieves a real stop.

## Critical impl rules

- **E-stop is owned exclusively by `mission_manager`**, with a monotonic
  **safety generation counter** (`_safety_generation`,
  `mission_manager_node.py:530`), incremented every time the latch is
  asserted (`_assert_emergency_stop`, line 1572). Release requires the
  matching generation via the `ReleaseEmergencyStop` service
  (`src/mission_manager_interfaces/srv/ReleaseEmergencyStop.srv`) — a plain
  `/emergency_stop=false` publish is not how release works.
- **2026-08-31 (today, commit `7ac712b`, current HEAD) fixed a real bug**:
  `rover_backend`'s release call was reading the safety generation from the
  wrong `rover_state` section (`"safety"` instead of `"mission"`), so
  `expected_manager_generation` was always `-1` and every release was
  unconditionally rejected — mission start was silently blocked
  (`src/rover_backend/rover_backend/ros_bridge.py:3356`, one-line fix). If a
  mission won't start and the estop won't release, this class of bug is
  worth checking for again.
- **`rover_backend` defaults safe on process start**: `/emergency_stop=true`,
  `/mission_enable=false`, no mission auto-prepared
  (`src/rover_backend/rover_backend/main.py:13, 420`).
- **Spray interlocks** (`src/spray_controller/spray_controller/spray_controller_node.py`):
  requires `require_px4_armed` + `require_px4_offboard`; any e-stop,
  mission-disable, OFFBOARD loss, disarm, stale control input, or
  marking-hold loss during a spray forces an immediate release-recovery path
  (docstring lines 27-28, callback line 537-540); faults **latch** and need
  an explicit `/spray/reset_fault`; a persistent journal prevents re-spray
  after a mid-spray crash. Independent hard watchdog `hard_press_timeout_sec
  = 5.0`.
- **Spray point-gating lives in `mission_manager`**: ⚠ updated 2026-09-27.
  The stop/marking tolerance is **20 mm** since `fcc3785` (2026-09-19:
  `marking_tolerance_m` and RPP `waypoint_tolerance_m` both 0.03 -> 0.02).
  Since `9c4aa8c` spray and COMPLETED are authorized **only** by the raw-GNSS
  gate: raw RTK-FIXED radial vs the surveyed point <= 30 mm
  (`raw_gnss_spray_tolerance_m: 0.030`, 0.5 s window since `737cf63`). The
  rover must be stationary (`stationary_speed_tolerance_mps: 0.01`). The
  2026-08-31 values this bullet used to quote (30 mm,
  `arrival_settle_sec: 0.30`, `marking_hold_sec: 3.00`) were the live capture
  of that day; read `rover.launch.py` for current ones.
- **Symlink-install is confirmed**: the local Mac `install/` tree's
  egg-links point at the Jetson's own build path
  (`/home/flash/rover_ws/build/<pkg>`), for `jetson_4wd_control`,
  `mission_manager`, `rpp_controller` at least. Editing a `.py` under
  `<pkg>/<pkg>/*.py` for any of the 9 `ament_python` packages takes effect
  on the **next node restart, no `colcon build` needed** — unless
  `setup.py` (entry points) or `package.xml` (deps) changed.
- **`mission_manager_interfaces` is the one `ament_cmake` package**
  (generates `ReleaseEmergencyStop.srv` and, since 2026-09-30,
  `RestoreMissionRuntime.srv`). A change to that `.srv` genuinely
  needs `colcon build --packages-select mission_manager_interfaces`, plus a
  rebuild/restart of every consumer (`mission_manager`, `rover_backend`) —
  this is the one real exception to the symlink-install shortcut above.

## OFFBOARD / safety-net differences from the 3WD sibling — read before assuming parity

- **`NAV_RCL_ACT=1`, `NAV_DLL_ACT=0`** in both local param exports — this
  rover does **not** disarm on RC-loss or datalink-loss the way the 3WD
  rover does (`NAV_RCL_ACT`/`NAV_DLL_ACT`=6/Disarm there). No code in `src/`
  compensates for this. **Do not assume a dropped link auto-disarms — it
  doesn't, on the params captured so far.** If this matters for a task,
  flag it rather than assuming the 3WD failsafe posture.
- The "≥2 Hz before OFFBOARD" and "gap → failsafe" rules from the 3WD
  project still apply in spirit (`cmd_vel_bridge` streams 50 Hz
  unconditionally, and RPP commands older than 0.25s are dropped to zero —
  see above) but the actual PX4-side failsafe response to a stream gap
  hasn't been separately confirmed here.

## RTK — confirmed live and working, but the manager code has open bugs

- Architecture: `rtk_correction_bridge`'s `ntrip_to_px4_node.py` is a full
  NTRIP client (RTCM3 CRC-24Q parsing, TLS-capable socket, NMEA GGA
  round-trip) — richer than the 3WD sibling's standalone script. Caster
  host/port/mountpoint/credentials are injected out-of-band by
  `rover_backend` (`ntrip_to_px4_node.py:83-101`), never plain ROS params —
  don't go looking for them in `ntrip_to_px4_node.yaml`, that file only has
  `health_heartbeat_sec` / `health_log_period_sec`.
- Lifecycle is owned by `rover_backend` (`RtkRuntimeOrchestrator` /
  `RtkManagerCore` / `RtkProcessAdapter` in `src/rover_backend/rover_backend/`).
  **`audit_report.md` (dated today, 2026-08-31) scores this "NEEDS FIX",
  88/100, and blocks Phase 3B/4/7 work on it** until two P1s are fixed: broad
  exceptions in `_execute_spawn`/`_execute_stop`
  (`rtk_runtime_orchestrator.py` ~lines 160, 174) can strand the FSM
  permanently, requiring a full backend restart to recover. One P2: the
  MAVROS-ready gate checks only `state.connected`, not that the RTCM
  injection topic actually has a subscriber — the audit recommends
  `count_subscribers('/mavros/gps_rtk/send_rtcm') > 0` instead.
- **Confirmed live and FIXED tonight** (2026-08-31, 19:16-19:18 IST,
  `bags_jet/evening_session_20260831/mission.csv_20260831_191653/telemetry.csv`):
  `gps_fix_type=6`, 26-33 satellites, `rtk_state=FIXED`, correction age near
  0. Independently re-confirmed by direct bag decode on a different bundle
  the same evening (`GPSRAW.fix_type=6`, 15 sats, `h_acc=2cm` — see the
  `analyse-missions` skill §2/§3). **Do not assume RTK needs setup here** —
  an old memory note about a project called `4WD_SERVER` said NTRIP wasn't
  configured; that note is about a *different, unrelated repo* (see
  "Not this project" below), not this one.
- **Since 2026-09-22 (`STEERING_FIX_V4`) `trajectory_generator` no longer
  gates placement on RTK.** Placement only projects surveyed lat/lon through
  PX4 `gp_origin`, so it requires frame evidence only (connected FCU
  session, current-session origin, fresh fused global + local odom, receive
  skew, finite values, origin residual). The fix-type/correction checks are
  gone from placement, and the `rtk_stable_sec` dwell **and parameter were
  removed** (also from `rover.launch.py`); RTK quality is reported as
  `rtk_diagnostic` in
  `/trajectory_generator/status`. **The RTK motion gate is Mission Manager's
  `_rtk_motion_ok()`** (fresh `fix_type == 6` at START/RESUME/NEXT), pinned by
  `src/mission_manager/test/test_rtk_motion_gate_contract.py`. A trajectory
  can be READY while RTK is FLOAT; that must not be read as "safe to move".
- **Runtime RTK policy while RUNNING (operator-approved 2026-09-26, not yet
  field-tested)** — `mission_manager/rtk_runtime_policy.py`, wired in
  `_monitor_runtime_rtk()` / `_monitor_rtk_float_pause()`. Before this, FLOAT
  was warning-only and the rover kept driving **and marking** on a FLOAT
  position. Now: fresh FLOAT (fix_type 5) → stop at once, pause reason
  `RTK_FLOAT`, **auto-resume** after FIXED holds 3 s continuously (never
  changes PX4 mode or arms; requires PX4 still OFFBOARD+armed, E-stop released,
  motion health OK, else manual). DGPS/3D/2D/no fix or GPSRAW stale 10 s →
  `RTK_LOST`, **manual** Resume (also if it happens during a FLOAT pause).
  FLOAT pause not auto-resumed within 120 s → `RTK_FLOAT_TIMEOUT`, manual. A
  spray press already in progress finishes before the FLOAT pause. The
  tablet only knows `RTK_LOST`, so a FLOAT pause shows "RESUME BLOCKED".

## Live-captured params (2026-08-31 evening, matches HEAD `7ac712b`)

> ⚠ **Several of these moved on 2026-09-01.** For `cruise_speed_mps`,
> `deceleration_distance_m`, `acceleration_startup_ceiling_mps`,
> `precision_pivot_stop_speed_tolerance_mps` and the new
> `post_pivot_reanchor_all_legs`, use the table in §"2026-09-01 field
> session" instead. Everything else below still held as of that session.

> ⚠ **Also moved since:** `waypoint_tolerance_m` and `marking_tolerance_m`
> are **0.02** since `fcc3785` (2026-09-19); every "30 mm latch" in this table
> and in the dated sections below describes the 30 mm build of its day. The
> raw-GNSS spray gate (`9c4aa8c`) is still 30 mm.

Pulled from `bags_jet/evening_session_20260831/mission.csv_20260831_191653/params/*.yaml`
— **prefer this over any static `.params` export in `px4-params/`**, which
is untracked/gitignored and has at least one internal contradiction (an
"Aug-27 VETRI" export has `EKF2_GPS_YAW_OFF=0.0`, a "Jul-31 Satheesh" export
has `180.0` — the live capture matches the Satheesh export, treat that as
the one currently governing behavior).

| File | Key values |
|---|---|
| `rpp_controller.yaml` | `cruise_speed_mps: 1.0`, `acceleration_distance_m: 0.5`, `deceleration_distance_m: 0.5` / `deceleration_floor_speed_mps: 0.15`, `pivot_enter_angle_deg: 45.0` / `pivot_exit_angle_deg: 12.0`, `pivot_yaw_kp: 1.0`, `maximum_yaw_rate_radps: 0.2` / `minimum_yaw_rate_radps: 0.06`, `waypoint_tolerance_m: 0.03`, `line_tracking_lookahead_m: 0.55` (0.35-0.8 adaptive — landed **today**, commit `d38155d`; ⚠ **this is NOT the lookahead that runs** — see the 2026-09-04 control-path section), `xtrack_priority_enter_m: 0.015` / `exit_m: 0.008`. Most `precision_*` gates are OFF by default (`precision_speed_control_enabled`, `precision_terminal_enabled`, `precision_pivot_enabled`, `precision_tracking_control_enabled`, `precision_curvature_enabled` all `false`) — only `precision_guidance_enabled` and `geometry_tracking_enabled` are on, deliberately preserving "the production 30mm latch" (`rover.launch.py:490` comment). |
| `mission_manager.yaml` | `marking_tolerance_m: 0.03`, `arrival_settle_sec: 0.3`, `marking_hold_sec: 3.0`, `stationary_speed_tolerance_mps: 0.01`, `spray_required: true`, `spray_confirmation_timeout_sec: 7.0`, `waypoint_match_tolerance_m: 0.002` |
| `spray_controller.yaml` | `press_value: 1.0`, `release_value: 0.0`, `spray_duration_sec: 0.5`, `pre_spray_stable_sec: 0.25`, `hard_press_timeout_sec: 5.0`, `require_px4_armed: true`, `require_px4_offboard: true` |
| `trajectory_generator.yaml` (`rtk_stable_sec` removed 2026-09-22) | `required_gps_fix_type: 6`, `rtk_stable_sec: 3.0`, `max_correction_age_sec: 2.0`, `interpolation_spacing_m: 0.05`, `localization_mode: shadow`, `frame_id: map` |
| `cmd_vel_bridge.yaml` | `command_timeout_sec: 0.25`, `backend_heartbeat_timeout_sec: 1.5`, `maximum_speed_mps: 1.0` |

FCU (live capture, same bundle): ⚠ `EKF2_GPS_P_NOISE=0.5` — **STALE, this is
0.01 on the vehicle**; confirmed in the live bag capture of two separate
2026-09-04 bundles and in the 2026-09-04 audit. See the 2026-09-05 section.
`EKF2_GPS_V_NOISE=0.3`,
`EKF2_GPS_CTRL=15`, `GPS_YAW_OFFSET=180.0`, `COM_RC_IN_MODE=0`. ⚠
**`PWM_AUX_FUNC5=0` in the live capture, but `spray_controller_node.py:7`
comments claim `PWM_AUX_FUNC5=301`** — this is an open, unresolved
discrepancy between code comment and live FCU state, not yet root-caused.
Don't trust either source alone for the spray AUX function until it's
reconciled.

## 2026-09-01 field session — speed contract + pivot walk

Nine missions across the afternoon, on branch `feat/rtk-injection-v2`.
Everything here is measured from that session's bags and from PX4 ulogs, not
inferred. Bags in `bags_jet/Sep_01/P1_logs/`; ulogs in
`~/Documents/QGroundControl Daily/Logs/4WD/speed_0.4/`.

### The OFFBOARD contract is velocity-only — there is no yaw command

`/mavros/setpoint_raw/local` carries `type_mask = 3527 (0xdc7)`, which sets
**both `IGNORE_YAW` and `IGNORE_YAW_RATE`**; the `yaw` and `yaw_rate` fields
are identically zero in every message of every bag. The entire contract is
VX/VY in `FRAME_LOCAL_NED`. **PX4 derives heading from the velocity vector
itself.** Do not go looking for a yaw command, and do not add one — PX4
velocity OFFBOARD discards `trajectory_setpoint.yawspeed` anyway (the 3WD
sibling hit the same structural limit and documented it).

### Speed shortfall — ROOT CAUSE: proportional-only droop, PROVEN from ulog

Commanded 0.40 m/s, achieved ~0.235 m/s. PX4 (v1.16.2, git
`54f0455ffcd755534539a7cf33a09a20bf71d29d`) internally regulates the **full
0.400 m/s setpoint — no clamp** — and outputs 0.264 normalized throttle,
which reconstructs its own control law to floating point:

```
throttle 0.264 = FF 0.211 (= 0.4 / RO_MAX_THR_SPEED 1.9)
               +  P 0.053 (= RO_SPEED_P 0.3 x error)
               +  I 0.000 (RO_SPEED_I was 0)      residual 3e-8
```

`RO_SPEED_I = 0` is a proportional-only loop, which **cannot** remove
steady-state error. **Saturation was ruled out**: max motor command 0.705
normalized including pivots, 0% of samples at any limit, PWM 1150-1850us
against 1000/2000 bounds, mapping `1500 + 500*control` with no downscaling.

**Fix applied same session: `RO_SPEED_I = 0` -> `0.2`. Achieved went
0.235 -> 0.37 m/s against a 0.40 command (59% -> 92%).**

Earlier in the session `RO_SPEED_LIM` was `0.40` and DID bind: a 1.00 m/s
mission command produced only 0.22-0.30 m/s, which only fits a 0.4 clamp
(without it the loop would have reached ~0.75 throttle). It was raised to
`1.20` before the 17:34 runs. **The ulog is authoritative for FCU params —
a QGC export read 0.4 while the ulog for the same runs read 1.2.**

### Drivetrain nonlinearity — RO_MAX_THR_SPEED is UNRESOLVED

Measured throttle -> speed, position-derived, non-pivot samples only:

| what | value |
|---|---|
| deadband ends | ~0.10-0.15 normalized throttle |
| local slope, 0.20-0.30 throttle | **1.25 m/s per unit throttle** |
| local affine fit | `speed ~= 1.248 * throttle - 0.103` |
| motor breakaway (command) | **0.143-0.219 m/s** |

`RO_MAX_THR_SPEED = 1.9` and the operator states full-throttle speed really
is ~1.9 m/s — but no log contains throttle above **0.277**, so that is
unverified from data, and the local slope is only 1.25. Both can be true if
the plant is strongly nonlinear, which is the worst case for a single linear
feedforward constant: one value cannot serve both 0.4 and 1.0 m/s.
**Settle it with a MANUAL-mode bench sweep** (`RO_SPEED_P=0`,
`RO_SPEED_I=0`, held throttle 0.10->1.00, logging position, motor outputs
and actual traction-battery current) **before pushing cruise above ~0.6.**
The ~0.32 A current channel in the ulog does NOT measure the Sabertooth
traction supply, so these logs cannot assess voltage sag.

### `RD_MAX_THR_YAW_R` is m/s, NOT rad/s — the yaw axis is fine

It is a **wheel-speed difference in m/s**. PX4 computes
`normalized_diff = (yaw_rate * RD_WHEEL_TRACK / 2) / RD_MAX_THR_YAW_R`, so
`0.30` yields a full-differential yaw rate of `2*0.30/0.63 = 0.952 rad/s`.
The rover sustains 0.66 rad/s (37.8 deg/s) at only ~0.60-0.70 differential
throttle, against `RO_YAW_RATE_LIM = 40 deg/s`. **The yaw feedforward is
correctly sized and does not saturate.** An earlier claim in this session
that it was ~2.5x oversized was a unit misreading and is retracted.

### Pivot settle — the stationary gate was reading a ringing estimator

`_chassis_stationary()` (`legacy_alignment.py:519`) gates on
`current_speed_mps = hypot(twist.linear.x, twist.linear.y)` off
`/mavros/local_position/odom` (`rpp_controller_node.py:3465`). That EKF
velocity has a **stationary noise floor of median 0.0233 / p99 0.0535 m/s**
(1221 samples where the rover genuinely moved <10 mm/s) and **rings through
zero for seconds after a pivot** — `hypot()` rectifies the negative lobe into
an apparent "rebound". At the old 0.030 gate, **33.4% of genuinely
stationary samples read as MOVING**.

Release always landed at exactly `(first sustained gate crossing) + 1.20 s`
— the `settle_sec 0.20 + post_settle_hold_sec 1.00` budget was never wrong.
Raising the gate `0.030 -> 0.060` cut estimator wait from 1.4-6.6 s to a
uniform ~1.2 s per pivot, **~10.4 s off a 4-point mission**. The yaw-rate
half of the gate never binds: post-stop `|yaw_rate|` is <=0.0085 rad/s
against a 0.050 threshold.

### Pivot walk — every pivot displaces the rover 300-600 mm

Measured net displacement during ALIGNMENT, 26 pivots: **363-513 mm when the
rover pivots, 37 mm when it does not** (one leg in run 162110 never pivoted —
a clean natural experiment). Much of it is the GPS antenna swinging through
its lever-arm arc; `EKF2_GPS_POS_X/Y = 0` so EKF2 treats the antenna as the
vehicle origin, and the antenna doubles as the spray-nozzle reference, so the
lever arm cannot be compensated without moving spray targeting.

Only the C->P1 entry leg used to rebuild its line from the post-pivot
position, and it was the only leg landing inside the 30 mm marking latch:

```
    bag   gate  reanchor |      P1       P2       P3       P4 | completed
 154640   0.03        no |   -72.9   -190.6   -165.8    -89.7 | 0/4
 162110   0.06        no |    +4.4   -178.6   -171.7   -133.7 | 1/4
 165849   0.06       YES |    -6.0   -183.7    -15.9    +18.5 | 3/4
 170044   0.06       YES |    -9.6   -145.8    -10.7    +10.5 | 2/4
```

Reanchoring every leg is safe because **nothing is painted between marking
points**: `marking_indices` fire only at the surveyed nav_path indices
(0/44/92/133 on this mission) and `/marking_active` went true exactly once,
for 0.84 s. The inter-point line decides how the rover ARRIVES, not what it
marks; the goal is never moved and `/nav_path` is never modified.

⚠ **Precision guidance derives its correction from the `/nav_path` geometry
projection, not from the path being tracked.** Leaving it with bearing
authority on a reanchored leg steers the rover back onto the surveyed line
and silently undoes the reanchor. Authority is now keyed on
`following_runtime_line` (true for the entry leg AND any reanchored leg), not
on `first_approach`. There is a test that fails if any guidance-authority
site goes back to keying off the leg index.

### As-run params, 2026-09-01 18:04 (run `mission.csv_20260901_180445`)

| where | values |
|---|---|
| `rpp_controller.yaml` | `cruise_speed_mps: 0.4` (staged bring-up 0.4->0.6->0.8->1.0), `acceleration_distance_m: 0.5`, `acceleration_startup_ceiling_mps: 0.25`, `deceleration_distance_m: 1.0`, `deceleration_floor_speed_mps: 0.15`, `precision_pivot_stop_speed_tolerance_mps: 0.06`, `post_pivot_reanchor_all_legs: true`, `segment_alignment_speed_mps: 0.4` |
| FCU (from ulog) | `RO_SPEED_LIM: 1.2`, `RO_SPEED_P: 0.3`, **`RO_SPEED_I: 0.2`** (was 0), `RO_SPEED_TH: 0.02`, `RO_MAX_THR_SPEED: 1.9`, `RO_ACCEL_LIM/RO_DECEL_LIM/RO_JERK_LIM: -1` (all disabled), `RO_YAW_P: 1.0`, `RO_YAW_RATE_P: 0.5`, `RO_YAW_RATE_I: 0.05`, `RO_YAW_RATE_LIM: 40 deg/s`, `RD_MAX_THR_YAW_R: 0.30 m/s`, `RD_WHEEL_TRACK: 0.63`, `THR_MDL_FAC: 0` |

### Commits landed this session

`b350d7a` pivot-settle gate 0.030->0.060 · `aa1f3fc` bag decoder +
pivot-settle report tooling · `4b8fe1b` reanchor every leg · `e638471`
acceleration bootstrap ceiling 0.08->0.25 · `19ebc7c` decel window
0.50->1.00 m · `943c17c` bounded cruise speed, staged at 0.40.

⚠ A teammate's commit (`d9cc15f`, SHARINI, `rover_backend` only) landed
mid-session and rebased two of these, so hashes moved once.

### Open, unresolved

- **P2 terminal swing.** With the reanchor in, every leg now STARTS on its
  line (-2 to -28 mm) but P1->P2 still arrives 146-184 mm off. Its terminal
  swing is -220/-237 mm vs 17-65 mm on every other leg, and it is the only
  leg reaching STOP at `distance_to_goal` 0.15-0.19 m rather than 0.02-0.03.
  `along_remaining` went NEGATIVE while cross-track was 184 mm, i.e. it
  crossed the goal plane beside the point. Confirmed real motion, not a
  reference-line change. The 3WD project has hit and solved two things that
  look like this — the 1/L bearing-gain spike as the aim point closes, and a
  bounded along-track arrival predicate — see `PX4_DXP/CLAUDE.md` and
  `src/rpp_controller_node.py` there.
- **`RO_MAX_THR_SPEED`** — see the nonlinearity section above.
- **`RD_TRANS_TRN_DRV = 0.10472 rad = 6 deg`** on the FCU while
  `pivot_exit_angle_deg = 12.0` in `rpp_controller.yaml`. Live mismatch,
  predates this session, unexplained. (`RD_TRANS_DRV_TRN` = 45 deg still
  matches `pivot_enter_angle_deg`.)
- **`rpp_controller` param dump timing out** (`timeout_after_15s` in
  `manifest.json` -> `as_run_config.ros_params.errors`) on 3 of 9 runs,
  costing bag provenance. Not root-caused.

## 2026-09-03 evening dataset — stop accuracy, and two tooling traps

Eight runs (`log_77/79/82/87/89/98/100/103` paired with
`mission.csv_20260903_{165719,165926,171227,171757,172119,174005,174203,174520}`),
43 waypoints, under
`~/Documents/QGroundControl Daily/Logs/4WD/Madhavaram/Sep_03/evening_run_2/`
(`Bags/` and `Ulogs/` side by side). Raw GNSS was **RTK FIXED for 100% of
every run**, 16-18 sats, h_acc 15 mm.

### ⚠ `scripts/analyze_mission.py` projects on the WGS84 ELLIPSOID; PX4 uses a SPHERE

`_geodesic_ne_m` / `_metres_per_degree` are ellipsoidal. PX4 (and
`src/trajectory_generator/trajectory_generator/localization_frame.py`) project
geodetic to local NED on a sphere of `R = 6 371 000 m`. The disagreement is
**range-proportional: +0.51% north, -0.13% east** at this site.

This produced a convincing 847-888 mm "local-frame offset" on the runs whose
`gp_origin` sat 165 m from the work area, and 10-52 mm on the runs where it
was close. Re-projecting spherically matched the controller's own `goal_x`/
`goal_y` to **0.0 mm on all 43 goals** — there was no rover bug.

**Rule: never compare an absolute local-frame coordinate against an
ellipsoid projection of a lat/lon.** Either project spherically, or (better,
and what `survey_truth.py` does) take the difference *around* the target so
the earth model cancels — at 40 mm error scale a 0.5% model gap is 0.2 mm.

### ⚠ EKF-vs-raw-GNSS separation is transport latency, not estimator bias

Naively differencing `/mavros/global_position/global` against
`/mavros/global_position/raw/fix` gives p50 22-49 mm and looks like an
estimator error. Split by speed it is **45-65 mm above 0.30 m/s and only
11-15 mm at rest** — at the measurement floor. A constant **-20 to -60 ms**
shift of the raw stream collapses the median to 15 mm in every run. Parked
and marking, the EKF agrees with raw RTK. Always split by speed before
calling a position difference a bias.

(Raw lat/lon are int32 * 1e-7 deg = an **11.06 mm quantum**, so single-fix
survey truth carries ±5.5 mm. `/mavros/gpsstatus/gps1/raw` and
`/mavros/global_position/raw/fix` are byte-identical in position — verified
650/650 samples — so GPSRAW is preferred: it carries position and fix quality
in one atomic message.)

### Why the rover stops short — measured, and only partly fixable by tuning

**35 of 43 waypoints stopped SHORT of the surveyed point** (raw RTK truth),
median 18.3 mm, sign test p = 4.2e-5. The settled EKF agreed with raw RTK to
a median of 1.1 mm, so **the short stop is the controller, not the
estimator**.

`TerminalStopRegulator._braking_output` commands
`sqrt(2 * conservative_decel * (along_remaining - brake_margin))`. It crosses
the measured 0.143 m/s motor breakaway at
`brake_margin + 0.143^2/(2*decel)` — **28.6 mm** at the as-run 0.015/0.75.
Inside that distance the command is below what the drivetrain can turn the
wheels with, so the rover stalls: median advance after the last non-zero
command was **-0.2 mm**.

There is a second, separate effect worth knowing: during hard braking the EKF
along-track estimate runs **transiently optimistic by ~25 mm** and
self-corrects over ~1.5 s (174520/P0001: `along_remaining` read -0.9 mm, then
relaxed back to the true +24.5 mm). The zero latch is one-shot, so the stop
decision is taken on that transient. The *settled* estimate is accurate; the
*decision-instant* estimate is not.

**2026-09-04: `radial_stop_brake_margin_m` 0.015 -> 0.003** (d_break 28.6 ->
16.6 mm). `radial_stop_conservative_decel_mps2` deliberately left at 0.75 —
raising it also shrinks d_break but brings BRAKE_PROFILE entry closer to the
goal, which is what caused the 2026-09-02 35-50 mm coast overshoot. Staged
next step if the run lands short with no overshoot: 0.75 -> 1.00.

⚠ **Do not expect tuning to reach zero.** The measured pair bounds it: ramp to
zero -> ~18 mm short (2026-09-03); hold the 0.15 m/s floor -> 35-50 mm past
(2026-09-02). The rover cannot be commanded below ~0.15 m/s and cannot stop in
under ~35 mm from it, so **the point is outside the reachable set of any
static profile**. Landing on it needs settle -> re-measure -> bounded creep
retry above breakaway, which is a controller change, not a parameter.

### Survey truth is now computed and reported (report only — RPP still gates)

- `trajectory_generator` publishes the uploaded lat/lon on
  **`/trajectory_generator/survey_targets`** (latched String/JSON,
  `dyx4wd/survey_targets@1`). Only place un-projected mission coordinates
  leave that node; nothing steers from it.
- `mission_manager` buffers raw GPSRAW fixes and computes physical stop
  accuracy in **`src/mission_manager/mission_manager/survey_truth.py`**
  (pure, ROS-free, 17 tests seeded with real Sep-03 coordinates).
- It appears as `accuracy["survey"]` in `/mission_manager/point_event`, in
  `point_accuracy_snapshots` in the mission summary, and as
  `point_survey_snapshots` + `survey_truth_*` health in
  `/mission_manager/status`.
- **The 30 mm latch, spray gating and the point verdict are unchanged** and
  still run on RPP. Survey truth is a second, independent measurement — it is
  allowed to be `available: false` (degraded fix, window not stationary,
  local-coordinate mission) without affecting anything.
- Sign convention matches the RPP report: `along_track_error_mm` positive =
  SHORT of the target; `cross_track_error_mm` positive = RIGHT of approach.
- Validated by replaying all 8 bags through the shipped module: 40/43 within
  6 mm of the independently measured value. The 3 that differ by 7-17 mm are
  the points where the rover kept moving 15-66 mm *after* the latch, so the
  two numbers are different instants, not a disagreement.

Judged on physical truth, 18/43 made the 30 mm latch. RPP's own verdict
agreed on 37 of 43 — **3 false accepts and 3 false rejects**.

## 2026-09-04 — verified control-path facts (read before touching tracking)

Established by reading the live source and cross-checking against the Sep-03
and Sep-04 bags. Each item says how it was verified, because two plausible
readings were **wrong** and are recorded here so they are not re-derived.

### The lookahead that actually governs is time-based, not `line_tracking_lookahead_m`

`precision_guidance_enabled: true`, so `guidance.py:11` computes

```
lookahead = clamp(lookahead_time_s * speed + xtrack_lookahead_gain * |xtrack|, min, max)
```

with `precision_lookahead_time_s: 0.9`, `precision_xtrack_lookahead_gain: 0.0`,
`precision_lookahead_min/max_m: 0.2 / 1.0`. Measured live: **0.936 m p50** at
1.03 m/s — exactly `0.9 x 1.03`. `line_tracking_lookahead_m: 0.55` and
`xtrack_priority_lookahead_m: 0.55` describe a path that does not run while
precision guidance is on. Tune `precision_lookahead_time_s`, not either 0.55.

Note the gain term would *lengthen* the lookahead as error grows, so raising
`precision_xtrack_lookahead_gain` is not a fix for poor convergence.

### The OFFBOARD contract has no heading authority — this is the structural one

`cmd_vel_bridge` sends `type_mask = 3527` (`IGNORE_YAW | IGNORE_YAW_RATE`) with
`msg.yaw = 0.0` and `msg.yaw_rate = 0.0` (`cmd_vel_bridge.py:59, 308`). PX4
derives heading from `atan2(vE, vN)` on its own. The controller can only steer
by rotating the velocity vector and waiting for PX4 to infer the turn.

Measured consequence: command-bearing to actual-heading lag is **1.00 s**
against EKF attitude and **1.20 s** against course-over-ground derived from raw
GNSS alone (correlation 0.25 at zero lag rising to 0.92 at ~1.0 s, so the peak
is real). That delay is **equal to the 0.9 s lookahead time** — the classic
marginal-stability condition for pure pursuit, and it matches the observed
non-convergent cross-track (Sep-04: 24.1 mm entering the cruise, 19.2 mm
leaving it, 27 of 37 legs crossing the line).

The 3WD sibling does not do this. `PX4_DXP/src/twist_to_setpoint_node.py`
publishes `type_mask = 2503` with an explicit `msg.yaw = atan2(v_n, v_e)` every
cycle, and 455 when a yaw-rate feedforward is live. Its own comment states the
reason: PX4's derived-yaw path "lags on turns". ⚠ That repo's CLAUDE.md
separately claims velocity OFFBOARD *discards* `trajectory_setpoint.yawspeed`,
so the explicit **yaw** (2503) is the demonstrated part and the **yaw_rate**
(455) is not — do not port 455 on the strength of the 3WD tree alone.

### The post-pivot reanchor is CORRECT in source, and UNOBSERVABLE in bags

The ordering documented above is right — this was checked because a bag-timing
reading suggested otherwise, and that reading was wrong:

- `legacy_alignment._settle_certificate_and_hold` emits `REANCHOR_ZERO` only
  after `_chassis_stationary_debounced` passes and the `settle_sec` dwell has
  elapsed, i.e. **after** the pivot settles.
- `reanchor_runtime_path_after_pivot` anchors at `self.current_x/current_y`,
  the post-pivot position, and never moves the goal.
- When it succeeds, `segment_runtime_reanchored` is set and
  `nav_solution = runtime_entry_tracking_solution(...)` with
  `path_label = "LEG_PATH"`, so `path_bearing` follows the reanchored line
  (`rpp_controller_node.py:9370-9382`).

⚠ **`/runtime_nav_path` is NOT the reanchor.** It is published by
`mission_manager` (`mission_manager_node.py:420`) and republished as goals
advance — its point count decreases monotonically through a mission
(1120 -> 995 -> 882 -> 758 -> 626 -> 496 on `mission.csv_20260904_141428`).
Timing it against pivots proves nothing about reanchoring. `rpp_controller`
publishes **nothing at all** when it reanchors, and `/rpp/pivot_debug` records
zero messages, so whether a given pivot actually reanchored cannot currently be
read from a bag. Fixing that observability is a prerequisite for any pivot or
reanchor work.

### What `/rpp/debug.cross_track_error_mm` is measured against

`goal_signed_cross_track` (`rpp_controller_node.py:9442`) is the perpendicular
offset from the line through the **goal** with bearing `path_bearing`, and
`path_bearing` follows the reanchored runtime line when one is installed. So
the reported cross-track IS against the line being tracked, not always against
`/nav_path` — the Sep-03/Sep-04 leg-convergence numbers are valid as tracking
error. (Precision guidance's own `signed_cross_track_m`, which *is* derived
from the `/nav_path` projection, is only substituted when
`not following_runtime_line` — `rpp_controller_node.py:10096-10102`.)

### Measured pivot behaviour (Sep-04, 6 real pivots)

Turns of 87-173 deg, walk 521-651 mm. Exit `|HE|` p50 **1.43 deg**, 5 of 6
under 4 deg — the pivot delivers good heading. Exit `|X|` p50 **21.0 mm**, only
2 of 6 inside 20 mm; if the reanchor had fired this should be ~0 by
construction, which is the open question the missing observability blocks.
The single 172.7 deg row reversal is the outlier on every axis: exit HE
7.57 deg, heading swing 15.3 deg, and 124.5 mm of genuine physical curvature
(raw GNSS, not a reference change).

⚠ Roughly half the *reported* post-pivot cross-track swing (63.5 mm p50) is the
reference line being rebuilt, not the rover moving — physical curvature from
raw GNSS is 6.1 mm p50. Do not read the reported swing as motion.

### `xtrack_priority_speed_mps` is a GUARD held neutral -- leave it that way

Set to `CRUISE_SPEED_MPS`, so the latch caps speed at exactly the speed it is
capping. Engaged on 62-90% of cruise cycles across both datasets and limits
nothing.

⚠ **This was repeatedly mis-filed as a defect to fix (2026-09-04 reports,
HANDOFF.md, and commit `96c35e6`, which set it to 0.60 and was reverted).
That framing is wrong and the revert was right.** Lowering the cap does not
merely slow the mission: lookahead is `precision_lookahead_time_s x speed`, so
capping 1.03 -> 0.60 m/s shortens the lookahead 0.936 -> 0.540 m and
**step-changes the pure-pursuit steering gain by ~1.7x** (gain goes as 1/L), as
a discontinuity, at the 15 mm engage threshold, in a loop that is already
marginally stable because the actuation delay equals the lookahead time. It
engages and releases with hysteresis, so the gain steps both ways mid-leg.
That is a swing generator, and the operator observed exactly that in the field.

Layering rule that follows: speed caps and xtrack priority are the **guard**
layer -- they bound damage when tracking is already wrong. The **fix** layer is
whatever makes the lateral loop converge (heading authority, then lookahead).
Guards must stay neutral while the fix is being measured, or they confound the
measurement and inject their own dynamics. Once tracking settles this guard is
not needed at all.

## 2026-09-05 — production task sheet is now the plan; EKF2/GNSS baseline

**`docs/4WD_CM_TRACKING_PRODUCTION_TASKSHEET_FINAL.md` is the current plan**
(committed `5686e21`), with `docs/2026-09-04_EKF_GNSS_Hardware_Audit.md`
(`b8aa859`) as its evidence base. It supersedes the ad-hoc "Step 0/1/2"
sequence still described in `HANDOFF.md`. Read the task sheet's P0 rules before
proposing anything here.

Two of those rules change how earlier work in this file should be read:

- **Runtime path is allowed only for the C -> P1 entry leg.** Every later
  `P* -> P*` leg must track its own fixed mission geometry. The post-pivot
  reanchor described in the "Control architecture" section above is therefore
  **diagnostic/recovery evidence only, not the production answer to pivot
  walk** — P7 removes it as a dependency. The instrumentation added in
  `60d4149` is still worth having under that reading.
- **Do not enable disabled precision states just because they exist**, and do
  not tighten estimator noise below measured receiver behaviour.

### Corrected as-run EKF2 configuration

⚠ **`EKF2_GPS_P_NOISE = 0.01`, not 0.5.** Confirmed in the live bag capture of
`mission.csv_20260904_141428` and `_143312`, and independently in the audit's
parameter file. The 0.5 in the 2026-08-31 table above is stale. This matters:
GNSS height observation noise is `max(receiver vacc, 1.5 * EKF2_GPS_P_NOISE)`,
so a 1 cm floor contributes only 15 mm — a confidently-wrong measurement is
accepted with a near-zero innovation.

| param | value | note |
|---|---|---|
| `EKF2_GPS_P_NOISE` | **0.01** | live-capture confirmed; was documented as 0.5 |
| `EKF2_GPS_V_NOISE` | 0.30 | live-capture confirmed |
| `EKF2_GPS_CTRL` | 15 | horizontal + altitude + 3D velocity + dual-antenna heading |
| `EKF2_HGT_REF` | **1 = GNSS** | GNSS is the long-term vertical authority |
| `EKF2_REQ_GPS_H` | **1.0 s** | firmware default is 10 s — aggressive |
| `EKF2_GPS_CHECK` | 831 | vertical-drift and horizontal-speed-offset checks DISABLED |
| `EKF2_GPS_DELAY` | 50 ms | |
| `EKF2_GPS_POS_Z` | -0.30 m | vertical lever arm set |
| `EKF2_GPS_POS_X/Y` | **0 / 0 — intentional, not a gap** | Rover pose origin = Main GNSS antenna = spray nozzle, by design (operator-confirmed 2026-09-09). This matches the 2026-09-01 pivot-walk section's independent finding ("the antenna doubles as the spray-nozzle reference, so the lever arm cannot be compensated without moving spray targeting"). There is no physical horizontal lever arm to measure here — do not "fix" this to a nonzero value. Earlier revisions of this file mischaracterized this as an unconfigured gap; that was wrong. |
| `EKF2_IMU_POS_X/Y/Z` | still open | Genuinely separate from the antenna/nozzle colocation above — this is the IMU-to-body-origin offset. Task sheet (`docs/4WD_CM_TRACKING_PRODUCTION_TASKSHEET_FINAL.md`) Task 1.2 still applies to this one; it does **not** apply to `EKF2_GPS_POS_X/Y`. |

✅ Re-confirmed live via NSH-over-SSH on 2026-09-09 (read-only `param show`,
see `docs/PX4_NSH_OVER_SSH.md`): `EKF2_GPS_POS_X=0.0000`, `EKF2_GPS_POS_Y=0.0000`,
`EKF2_GPS_POS_Z=-0.3000` — matches the uploaded `.params` file exactly. The
earlier caveat about this coming only from an unverified static export no
longer applies to `EKF2_GPS_POS_X/Y/Z`.

### The log_82 questions, resolved and unresolved

**Horizontal: RESOLVED — ordinary tracking/stop error, not a false position.**
Two independent methods agree. The audit searched every sample for
`RPP <= 20 mm AND raw > 50 mm` and found **zero**; its per-point raw radials
(56.7 / 82.1 / 24.4 / 16.3 mm) match the survey-truth module's
(54.9 / 83.6 / 27.2 / 15.6 mm) to a few mm. The error the operator saw is the
cross-track problem already characterised above.

**Vertical: OPEN — real receiver-native anomaly, root cause unproven.**
⚠ Likely the same failure mode as the 2026-09-08 B group (§2026-09-08): same
sign, same 100-150 mm magnitude, same "cleared by a power cycle". Read that
section before re-opening this one.
Raw GNSS altitude in the three runs **before** a rover power-off (`165719`,
`165926`, `171227`) sits 125-152 mm below the eight-run median; all five runs
**after** it sit within -2 to +52 mm. At the four piles `171227` shares with
later runs it reads 139/154/155/160 mm lower — 4 of 4 stops, same sign and
magnitude — while every other run agrees with every other at the same pile to
±36 mm. Throughout, the receiver reported `fix_type 6`, 17 satellites and
`h_acc 15 mm`. **A power cycle cleared it.** EKF2 did not create the offset but
was configured to trust GNSS as height reference with no independent protection
against a stable, confidently-reported absolute bias.

### ⚠ Comparing raw GNSS against fused global proves NOTHING about correctness

`/mavros/global_position/global` is derived from the raw fix. Under a
common-mode GNSS error both are wrong together and agree perfectly, so their
agreement measures internal consistency only. `fix_type` and `h_acc` are the
receiver's self-assessment and are exactly what is wrong in this failure mode.
Detecting a bad fix needs an **external reference** — revisiting the same
physical point across runs is the one available from these bags, and it is what
found the vertical offset. The 2026-09-03 "EKF-vs-raw separation is transport
latency" conclusion above is correct **and insufficient** for this reason.

### ⚠ Two traps in position-derived speed (both hit on 2026-09-04)

- **It cannot distinguish parked from pivoting in place.** Rotation produces
  near-zero net translation over a ~0.5 s window, so pivot samples land in an
  "at rest" bucket and carry the largest errors — this contaminated the
  2026-09-03 at-rest split (conservatively: true at-rest agreement is *better*
  than the 11-15 mm reported). Gate on yaw rate as well; the controller already
  does via `radial_stop_stationary_yaw_rate_radps`, the analysis scripts did not.
- **It uses bag RECEIVE time, which compresses under bursty delivery.** Three
  fixes arriving 1.0 ms apart while representing 0.3 s of real motion produced a
  spurious 2.61 m/s. `NavSatFix` carries a header stamp that `_p_navsatfix`
  reads and discards — use it instead.

## 2026-09-05 — Task 2.1 measured: GNSS/EKF noise (no param changed)

`docs/2026-09-05_Task_2.1_GNSS_EKF_Noise_Measurement.md`, tool
`scripts/gnss_ekf_noise_report.py`, raw output under `docs/measurements/`.
Stationary windows are detected from **actuator output + gyro only**, never from
GNSS or the EKF, so the scatter measured inside them is not circular.

| quantity (parked, RTK FIXED) | measured sigma |
|---|---|
| raw GNSS horizontal (2D) | **2.6 mm** |
| raw GNSS vertical | **4.8 mm** |
| raw GNSS velocity N/E/D | **7.9 / 8.3 / 10.6 mm/s** |
| raw GNSS yaw (dual-antenna) | **0.128°** |
| EKF local position x/y/z | **2.1 / 3.1 / 5.7 mm** |
| EKF local velocity x/y/z | **4.3 / 4.7 / 4.7 mm/s** |
| EKF yaw | **0.0095°** |

Four things worth carrying forward:

- **RTK error is time-correlated — averaging does not remove it.** Against a
  1/sqrt(tau) white-noise expectation at tau = 2 s, GNSS east is 3.2x high,
  GNSS altitude 3.5x, EKF y 5.2x, EKF z 5.1x (EKF y and z do not fall with tau
  at all). Repeatability at one revisited point across four runs is **42 mm
  vertical** against 4.8 mm within-stop jitter. **Never size a noise parameter
  off the 0.1 s jitter.**
- **Innovation gates have no detection authority here.** `estimator_aid_src_gnss_*`
  test ratios: p50 ~0.000-0.002, worst single sample anywhere **0.143**,
  **zero rejections** across 1390 samples x 4 aid sources, moving and stationary,
  100% fused. Zero EKF resets, 0/1390 failing `estimator_gps_status` checks. The
  `log_82`-class fault (stable, confidently-wrong fix) arrives with a near-zero
  innovation — catching it needs an external reference, not a tighter gate.
- **`EKF2_GPS_V_NOISE = 0.30` must not be lowered.** The receiver reports
  `s_variance_m_s` = 0.4 mm/s against 8-11 mm/s measured — it under-reports
  velocity accuracy ~20x, and PX4 takes `max(reported, EKF2_GPS_V_NOISE)`.
- **The EKF keeps moving ~1.7 s after the drivetrain goes neutral** (36 stops:
  p50 16 mm / p95 111 mm of further reported motion, p50 1.72 s to settle within
  10 mm), while the raw receiver settles in p50 0.09 s. `arrival_settle_sec` is
  0.30 s and the zero latch is one-shot, so the 30 mm marking verdict is taken
  mid-transient. Consistent with the 6/43 RPP-vs-truth disagreements on Sep-03.

⚠ **The Sep-04 mission bags have NO paired ulog** — the only Sep-04 ulog
(`log_25`) is 5.6 s. Every estimator-internal number above is Sep-03 only.
⚠ **Prefer the ulog over `GPSRAW` for GNSS position noise**: bag lat/lon carry
an 11.06 mm quantum (sigma 3.19 mm) that dominates a millimetre-scale
measurement; `vehicle_gps_position.latitude_deg` in the ulog is a double.

Still wanted before Task 2.2 closes: **a 20-30 min static log** (parked, armed,
RTK FIXED, full logging rate). It needs no bench and no mission, and it would
measure the correlation time directly instead of the tau <= 2 s floor above.

## 2026-09-08 — the ~150 mm RTK bias is receiver-side, and BeiDou is the lead

> ⚠ **SUPERSEDED on 2026-09-09 — read §2026-09-09 first.** The BeiDou lead, the
> reboot framing and the A-vs-B height argument in this section are retracted;
> `log_15` sits on a 210 mm-wrong fix with BeiDou at 99.7 %. What still holds:
> antenna ARP/PCV is not the discriminator, the error is in the receiver's own
> baseline, and the receiver cannot self-assess. The tool below is unchanged.

Full writeup: `docs/2026-09-08_Handoff_Septentrio_BeiDou_Wrong_Fix.md`. Dataset
`Estimator_Compare/` (A = 11 good ULogs 17:06-17:31, B = `log_47`/`log_48`
19:21-19:25). Everything below was decoded from the raw serial traffic PX4
captures in `gps_dump` (`SEP_DUMP_COMM=3`), not from `sensor_gps`.

**Tool: `python3 scripts/gps_dump_sbf_report.py [--reference targets.csv] <*.ulg>`**
— splits `gps_dump` into the two directions and decodes both: CRC-checked SBF
from the receiver, CRC-checked RTCM3 into it. This is the only way to see which
GNSS signals the RTK solution actually used, and the receiver's own ARP /
phase-centre flags. Layouts come from PX4's `sbf/messages.h`, ArduPilot's
`AP_GPS_SBF.h` and the mosaic-H v4.14.10 reference guide, each size-checked
against the receiver's `Length`/`SBLength`. ⚠ ArduPilot's `VectorInfoGeod`
declares `ReferenceID` as `u1`; spec and receiver both say `u2`, and the `u1`
layout silently shifts `SignalInfo` by one byte.

### Antenna ARP / phase-centre is NOT the cause of the good-vs-bad split

- **PX4 never configures the antenna.** `SeptentrioDriver::configure()` sends
  only `scs`, `sso`, `sdio`, `sto`, `srd`, `sga`, and `ssu` when
  `SEP_CONST_USAGE != 0`. No `setAntennaOffset`, no antenna type, no PCO/PCV,
  in **either** `SEP_AUTO_CONFIG` state. Antenna config is receiver NVRAM.
- `PVTGeodetic.Misc` is `0x50`/`0x60` in all 13 logs, good and bad, in the same
  proportion: bit 0 = 0 (baseline not known to point to base ARP), bit 1 = 0
  (rover PCO not compensated), bits 6-7 = 1 (ARP-to-marker offset is zero).
  Missing RTCM 1007/1008/1033 likewise identical in both groups.
- `BaseVectorGeod.Misc` bits 0 and 1 are both clear, so the spec's "accurate
  ARP-to-ARP baseline guaranteed only if both are set" is genuinely unmet —
  real, worth closing, worth a few cm, **not this defect**.

### What actually changed

The receiver was **power-cycled** between the groups (uptime 3780 s -> 2678 s)
and came back not using BeiDou. `PVTGeodetic.SignalInfo`:
`0x30220909` (with BDS B1I+B2I) in A, `0x00220909` in B. BeiDou used in 88-100%
of A epochs, **0.4% / 3.7%** of B epochs; `NrSV` 20 -> 15, VDOP 1.00 -> 1.64.

- **Not the corrections.** RTCM 1124 carried 8-9 BeiDou satellites in B at the
  same rate as A, yet `BaseVectorGeod.SignalInfo` reports no BeiDou corrections
  available. The receiver discarded them.
- **Not propagation.** BDS B2I (1207.14 MHz) was dropped while Galileo E5b at
  the *same* frequency was kept; BDS B1I (1561 MHz) dropped while GPS L1CA
  (1575 MHz) kept. Nothing frequency-selective does that.
- **Not PX4.** `SEP_CONST_USAGE = 0` means "constellation usage isn't changed"
  and the driver skips `ssu` entirely — the constellation set is receiver NVRAM
  in both auto-config states.

### The bad fix is a wrong integer ambiguity resolution

B is static (0.10-0.36 m span over 184-470 s), so 148 mm is measurement, not
control. `PosCovGeodetic` reports sigma_North ~9 mm against a 143-151 mm error
while holding `Mode = 4` / `Error = 0` — wrong by ~16 sigma. It drifts smoothly
+123 -> +153 mm over 11 parked minutes (a fixed wrong integer set through a
rotating geometry matrix; an antenna offset would be constant, noise random).
**Height reads 100-140 mm below the good group at the same pile** — matching the
still-open 2026-09-03 vertical anomaly (§2026-09-05) in sign, magnitude and in
being cleared by a power cycle. Probably the same recurrent failure mode.

Revised ranking: **#1 wrong ambiguity fix** (was #3), **#2 BeiDou loss as the
trigger** (was "geometry"), **#3 antenna ARP/PCV** (was #1).

### Sharpest open question

`log_20` ran on a constellation about as thin as B's (`NrSV` 14, HDOP 1.12,
BeiDou 19%) and stayed **RTK float for all 383 epochs** — it declined to fix.
B, similarly thin, declared FIXED and was wrong by 150 mm. Unexplained.

### Next moves (no field data needed for the first three)

1. Interrogate the receiver via NSH/web UI and record: `lif,Permissions` (is
   BeiDou licensed?), `gst`, `gsu`, `gao`, `lif,AntennaInfo`, `grd`, `gpm`, and
   `lif,error` (every log has `ReceiverStatus.RxError` bit 3 `SOFTWARE` set, in
   both groups, never explained; `lif,error` reports and clears it). Save to
   NVRAM so a power cycle cannot change it again.
2. Set `SEP_CONST_USAGE = 31` and `SEP_AUTO_CONFIG = 1` so PX4 commands `ssu`
   every boot and the constellation set stops depending on NVRAM. ⚠ `ssu` sets
   *usage*, not *tracking* — check both in step 1.
3. Add `ReceiverSetup` (5902), `ChannelStatus` (4013) and `SatVisibility` (4012)
   to a **second** receiver SBF stream (so auto-config can stay on) and
   `gps_dump` will capture them. `SEP_SAT_INFO` does **not** do this — the driver
   only copies the satellite count into `satellite_info`. Without those blocks no
   log in this project contains per-satellite azimuth/elevation/CN0.
4. Only then re-test: static placement at P0001, BeiDou confirmed in
   `SignalInfo`, same post-sunset window, and see whether 150 mm returns.

## 2026-09-09 — ROOT CAUSE: wrong integer ambiguity fix; it only changes on a re-fix

Full writeup: `docs/2026-09-09_Handoff_RTK_Wrong_Ambiguity_Fix_Root_Cause.md`.
Established over **every September ULog** (164 found, 142 unique, 119 with >20 s
of GNSS), not one A/B bundle.

**Inside one 98-minute log, at the same physical point, with nothing changed —
no reboot, no parameter, no configuration, no constellation change — the reported
position steps 192 mm, exactly on the only RTK ambiguity re-resolution in the log.**
`log_15_2026-9-8-14-10-54`, parked fixes within 0.3 m of P0001, split on its one
1093 s RTK outage (t = 2828 -> 3921 s):

| | fixes | height p50 | sats | vdop | `eph` | BeiDou |
|---|---|---|---|---|---|---|
| before the re-fix | 10 064 | **-87.487 m** | 18 | 1.30 | **15 mm** | 99.7 % |
| after the re-fix | 9 509 | **-87.295 m** | 19 | 1.14 | **15 mm** | 99.7 % |

Within each band the height wanders only tens of mm across many parked windows.
**Reported position is stable while a fix is held and steps by decimetres when the
fix is re-resolved.** `log_18` corroborates (-210 mm across a 170 s re-fix at one
spot). `log_47` is the control: 99.5 % fixed, no outage over 1.6 s, height held to
30 mm across 4 minutes — one fix, held, wrong.

Truth anchor: `log_12_2026-9-8-11-06-54` is verified good — parked 74.9 s, RTK
fixed, raw **10.1 mm** and fused **5.2 mm** from surveyed P0001, `fused - raw` =
N +6.1 / E -3.3 / **U -300.0 mm** (exactly `EKF2_GPS_POS_Z`), 0 innovation
rejections, 0/974 `gps_check_fail_flags`. It reads **-87.277 m** at P0001 and the
Sep-07 day median is **-87.261 m**. So log_15-before is ~210 mm low and log_47/48
~110-165 mm low, while log_15-after is within 35 mm.

### Retracted from the 2026-09-08 section

- The **reboot** is not the mechanism — five receiver boots on Sep-08, four came
  back with BeiDou (99.7/91.9/93.9/19->100 %), one did not (0.4/3.7 %). A reboot
  just forces a re-fix.
- **BeiDou is not necessary** — log_15 was 210 mm wrong with BeiDou at 99.7 %. Its
  absence in boot #5 stays a real, unexplained receiver-side anomaly (the base sent
  RTCM 1124 with 6-10 sv in every log all day) and a plausible aggravator, not the
  cause.
- The **A-vs-B height argument** was not evidence: log_15 swings 230 mm vertically
  at P0001 within one log and goes *below* log_47.
- **+148 mm North is not uniquely large** — log_15's long holds sit at dN +96..+108
  at P0001. And nothing in these logs measures where the rover physically was.

### Cleared by the September-wide sweep

- **RTCM injection is healthy everywhere**: 112/119 logs at exactly 6.0 Hz median.
  The 7 exceptions are 6 total outages (0 Hz, 0 % RTK: `log_83`/`log_84` 9-3,
  `log_49`/`50`/`51` 9-7, `log_31` 9-8) plus `log_1` at 4.8 Hz. Reduced correction
  rate is not a factor in any bad-position log.
- **Constellation content is measurable only on 17 logs, all 2026-09-08**
  (`SEP_DUMP_COMM` was 0 before that). Sep-01/03/04/07 cannot be checked at all.
- **2026-09-08 is the anomalous day, not the evening.** Within-day vertical spread
  at P0001: Sep-03 max **41.9 mm**, Sep-07 max **43.8 mm**, Sep-08 max **416.4 mm**,
  starting in the 10:30 session. Unexplained; no parameter can do it (the only
  GNSS-relevant diffs are `EKF2_GPS_CHECK` 831->829, `EKF2_GPS_YAW_OFF` 0->180 on
  log_47, `SEP_AUTO_CONFIG`, `SEP_DUMP_COMM` — and EKF2 params cannot move raw GNSS).

### What to do

1. **The only defence that matches the failure.** `eph` is 15 mm in both the correct
   and the 210 mm-wrong state, so no innovation/DOP/fix_type/sat-count gate can
   separate them — settled, not fixable by tightening anything. But the *transition*
   is observable: `fix_type` dropping below 6 and returning. `mission_manager` should
   treat an RTK re-fix as invalidating prior marking references and refuse to keep
   marking until the rover re-verifies against a known surveyed point.
2. **Quantify the exposure**: park at P0001, force ~10 re-fixes (cut corrections
   60 s, restore, repeat), log each settled position. Discrete clusters 100-200 mm
   apart confirm the mechanism. One hour, no bench, no mission.
3. **Fix less often wrongly**: restore BeiDou determinism (`lif, Permissions`, `gst`,
   `gsu`; `sst, all` + `eccf, Current, Boot` if tracking is off; `SEP_CONST_USAGE=31`
   with `SEP_AUTO_CONFIG=1` — ⚠ `ssu` sets usage, not tracking); ask for a nearer
   base or a VRS/MAC mountpoint (13.68 km is long for single-base integer fixing);
   close the antenna gap (`sao` with antenna type, base 1006/1007/1008/1033).

## 2026-09-10 — Direct Mosaic-H USB2 RTCM injection is now the production correction path

**Supersedes the correction-transport description in "RTK — confirmed live and working" above for
current architecture** (that section's NTRIP-client/lifecycle/bug content is otherwise still
accurate — this is additive, not a retraction). Full detail:
`docs/RTCM_DIRECT_INJECTION_AB_REVIEW_PLAN.md` (see its STATUS section), summarized in `HANDOFF.md`'s
2026-09-10 entry. Branch `feat/rtcm-direct-gnss-uart`.

An A/B-selectable transport now exists beside the legacy path, and both sides are individually
field-validated (Phase 5/6 PASS, 2026-09-10):

```text
Legacy A-side (still available, still passes): NTRIP → Jetson → MAVROS/PX4 → mosaic-H
Production B-side (this rover's active profile): NTRIP → Jetson → direct Mosaic-H native USB2
```

- Mosaic-H receiver serial `3804732`. USB mapping: `if02 → /dev/ttyACM1 → USB1` (diagnostics),
  `if04 → /dev/ttyACM2 → USB2` (production). Stable device path:
  `/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device_3804732-if04`. **`/dev/ttyACM0` is the
  Pixhawk FCU and must never be treated as the Mosaic-H.** This is a second logical serial port the
  receiver exposes over its own USB cable, not a separate wired UART.
- **This rover's active persisted RTK profile now runs `direct_inject = true`** against that device
  at 230400 baud, `desired_state` persisted `RUNNING` — direct USB2 is this rover's effective
  production correction path today. The software/schema factory default for a brand-new profile is
  still `direct_inject = false`; do not assume that default describes this rover.
- Backend startup reconciles from persisted `desired_state`, so a normal
  `ros2 launch rover_bringup rover.launch.py` restores RTK automatically once RTK has been explicitly
  STARTed once. ⚠ **Updated same day (2026-09-10):** this used to also require PX4/MAVROS to be
  connected before the worker would even spawn, in *either* injection mode. That MAVROS/PX4
  dependency has since been bypassed for `direct_inject=true` profiles —
  `build_production_runtime()` in `src/rover_backend/rover_backend/rtk_backend_lifecycle.py` now
  wraps the readiness provider so a direct-inject profile is treated as launch-ready without waiting
  on MAVROS at all. `RtkManagerCore`/`rtk_mavros_readiness.py` are unchanged; `direct_inject=false`
  still gates on MAVROS exactly as before. Verified at source/unit-test level only (3 new tests,
  full `rover_backend` suite green); not yet field-tested with PX4 actually absent. Detail in
  `docs/RTCM_DIRECT_INJECTION_AB_REVIEW_PLAN.md` §6.8.
- ⚠ **Changing any direct-injection field on a running profile forces persisted `desired_state` to
  `STOPPED`** (existing safety lifecycle contract) — RTK does not resume until an operator issues an
  explicit START again. A stationary direct-mode test once looked like a USB fault (no RTK, receiver
  stuck 3D lock) for exactly this reason; it was the STOPPED state from a prior profile edit, not
  hardware. See `HANDOFF.md` 2026-09-10 for the full incident.
- B-side field result: RTK FIXED ~113 s after `RUNNING`, `853` frames / `80524` bytes written,
  `0` write failures / CRC failures / invalid headers, `/mavros/gps_rtk/send_rtcm` silent throughout
  (proves exclusive routing, no dual RTCM path).
- **Accuracy boundary, do not skip:** this closes correction transport and RTK acquisition only. It
  does not prove surveyed absolute accuracy and does **not** solve the ~150-210 mm wrong-but-RTK-FIXED
  ambiguity behavior in the 2026-09-08/2026-09-09 sections above — that failure mode reports a clean
  `fix_type`/`eph` in both the correct and wrong state regardless of which transport delivered the
  correction. No A/B field accuracy comparison (Phase 7) has been run.
- Frontend (`DYX_GCS_Frontend`) cannot yet display/change `direct_inject` or select a USB device; its
  PATCH builder only sends changed fields, so editing other profile fields does not clobber the
  persisted direct-USB settings. Not a blocker for the current production profile.

## 2026-09-28 — Jetson hotspot network and spray manual-control API

**Network: the router is gone.** The Jetson runs its own WPA2 access point
`DYX-4WD` (NetworkManager `DYX-4WD-hotspot`, shared mode, autoconnect) and
keeps `192.168.3.101`, so the frontend and `backend.env` are unchanged.
Tablets get `192.168.3.x` by DHCP and share the Jetson's USB 4G internet
(they need it for Mapbox tiles). Runbook for provisioning a new rover:
**`docs/JETSON_HOTSPOT_NETWORK.md`** (`scripts/setup_jetson_hotspot.sh`,
`scripts/install_hotspot_qos.sh`, both run by the operator with sudo).
⚠ **Temporary dev exception:** `flash` has full NOPASSWD sudo via
`/etc/sudoers.d/99-flash-dev-nopasswd`, granted by the user for one month. The
persistent timer `remove-flash-dev-sudo.timer` deletes it at **2026-10-30
18:30 IST**, and the timer survives reboots. Check the file exists before
relying on it. Replace it with a rule restricted to `systemctl ... dyx-rover`
before shipping.

- ⚠ **Client internet is capped on purpose** (200 KiB/s down / 64 KiB/s up,
  all clients together, `mangle/DYX_HOTSPOT_QOS` via an NM dispatcher). Before
  the cap, forwarded client downloads (494 KiB/s) filled the Wi-Fi queue and
  the app stalled: Mac->Jetson ping worst 1459 ms, Jetson->tablet 785 ms. After:
  worst 90 ms under a download, tablet 6.8 ms avg. DDS multicast was measured
  and ruled out (3.5 KB/s). Do not remove the cap to "speed up" the map —
  use Mapbox offline packs instead.
- ⚠ **The Jetson kernel (`5.15.185-tegra`) has no HTB/fq_codel/cake/TBF/prio
  qdisc and no DSCP/TOS/CLASSIFY target.** Don't propose `tc` shaping or DSCP
  marking without building modules; `xt_hashlimit` is what exists.
- Setting regdomain `IN` did **not** raise txpower above 15 dBm at runtime
  (vendor driver). Range is the open field risk; external antennas are the fix.

**Spray: manual control + saved hold time.** `POST /api/spray/{on,off,test,
enable,disable,duration}` — full contract in
**`docs/SPRAY_MANUAL_CONTROL_API.md`**. Operator workflow: tune with
`/test {duration_s}`, save with `/duration {spray_duration_sec}`. The saved
value persists in `/home/flash/.ros/dyx_spray_settings.json` and overrides
`spray_duration_sec` in `rover.launch.py` on every start
(`spray_duration_source` in `/api/spray/config` says which applies). Manual
spray requires PX4 armed and no active mission; unit-tested only, not
field-tested; frontend buttons not wired yet.

### Marking tool LASER / SPRAY and default modes (2026-09-28, not field-tested)

**Factory default is now MANUAL execution + LASER marking** (was AUTO +
spray). LASER = AUX6 (`PWM_AUX_FUNC6=302`, `MAV_CMD_DO_SET_ACTUATOR`
param2), ON at each point that passes the same RPP + raw-GNSS gates as spray,
held with **no time limit** until the operator presses NEXT (steady when raw GNSS ≤ 30 mm; **blinks 0.5 s ON/OFF** when raw GNSS *measured* > 30 mm; OFF when the stop could not be measured); NEXT waits for a
PX4-acknowledged laser OFF before re-enabling motion. **Since 2026-09-29 the
final point does NOT wait for NEXT** (operator request): the mission
completes at once in every mode, and the completion E-stop turns the laser
OFF, so the final point's laser shows only momentarily. START refuses LASER + AUTO. Operator can pick SPRAY/AUTO per
mission and save defaults (`/home/flash/.ros/dyx_mission_defaults.json`).
**This rover's saved default is AUTO + SPRAY** (set 2026-09-30,
`defaults_source: saved`, survived a stack restart). The factory/launch
default is still MANUAL + LASER.
Full contract, PX4 params and API: **`docs/LASER_MARKING_MODE.md`**. ⚠ PX4
`PWM_AUX_FUNC6/MIN6/MAX6/DIS6` must be set (302 / 1000 / 2000 / 1000) before
the laser will move — not verified on the FCU yet.

### Voice announcements on a USB speaker (2026-09-29, not run on the rover yet)

`rover_backend` speaks important events in English / Tamil / Hindi
(`voice.py`, `voice_events.py`, `voice_phrases.py`, `voice_routes.py`;
hooks in five `ros_bridge` callbacks). This includes "Heading to target N",
"Target N done", and LASER "mark here" / "do not mark". The contract, engines
and setup are in **`docs/VOICE_ANNOUNCEMENTS.md`**.

Operational facts:
- Engines: Piper CLI (en, hi, offline), then edge-tts (Tamil, cloud), then an
  English fallback.
- Playback is ALSA direct (`aplay -D`), with no PulseAudio.
- Settings: `~/.ros/dyx_voice_settings.json`. Cache:
  `~/.ros/dyx_voice_cache`.
- One-time setup: `scripts/setup_voice.sh`.
- Speech is best-effort and must never block or raise into ROS callbacks.
  Keep it that way.
- ⚠ **Never `pip install --user` anything that pulls NumPy 2** (piper-tts,
  onnxruntime, …) on the Jetson. User-site NumPy 2 shadows ROS Humble's
  system NumPy 1.21, and `rpp_controller` crash-loops at import through
  `tf_transformations` → `transforms3d`. `colcon build` does not fix it.
  Voice tools live in the venv `~/.local/share/dyx-voice/venv`. If ROS
  nodes suddenly die at import, check
  `python3 -c "import numpy; print(numpy.__file__)"` first.

### Forward obstacle stop-and-wait (2026-09-29, sensor not fitted yet)

The new package `obstacle_monitor` reads a DYP A21 ultrasonic sensor on UART
and publishes `/obstacle/status`. Contract:
- **danger** (≤ 1.5 m): `mission_manager` pauses a RUNNING mission with
  reason `OBSTACLE`, **manual resume only**, and refuses START / RESUME / NEXT
  until the path clears. `cmd_vel_bridge` independently zeroes velocity.
- **warning** (≤ 3.0 m): spoken only.
- **dead sensor**: warning only, the mission continues (operator decision).
- **Off by default.** Settings are in `~/.ros/dyx_obstacle_settings.json`.
- Setup: `scripts/setup_obstacle_sensor.sh`. It's a new package, so it needs
  `colcon build` once.

Full contract: **`docs/OBSTACLE_DETECTION.md`**. Driving *around* obstacles is
the next feature and is not implemented.

## 2026-09-30 — realtime events, battery critical pause, power-cycle recovery

Branch `latency-hardening`, **fast-forwarded into `main` at `586e6fe`
(2026-09-30 22:18, user's decision)**. So `main` includes the MAVROS patch,
the relay, the radial20 fix and point_history, although the last two are not
field-tested yet. **Deployed on the Jetson as of 19:45 IST: `d2828a0`**
(later commits are docs only). Recovery, the battery pause, E-stop Release &
Resume and the power-cycle popup were exercised on the rover on 2026-09-30.
The radial20 fix and point_history are **not field-tested**.

- **`rtk_status` socket event** (`a21e6e2`). This is the GET `/api/rtk/status`
  body, emitted every 1 s, at once after any RTK profile/source/START/STOP
  mutation, and on connect. Built off the event loop.
- **`system_event` socket event** (`3ee0070`, `system_event@1`).
  - mission_manager's `_emit_system_event` publishes on
    `/mission_manager/point_event` **without a `point_id`**. Examples: OBSTACLE_*,
    RTK_FLOAT/PAUSED/RECOVERED, ODOM_PAUSED, ESTOP_*, MISSION_*,
    BATTERY_CRITICAL_PAUSE, MISSION_RECOVERED, SPRAY_*_MONITOR_ONLY.
  - Before this, ros_bridge only spoke them (voice).
  - They now share the ordered point-event queue with `event_kind: "SYSTEM"`.
- **Battery critical** (SHARINI `ef14c9a` supersedes my `3ee0070` filter):
  - `BatteryCriticalDebounce`: ≤ 23.80 V held for 5 s, with samples no older than 1.5 s.
  - The action is **PAUSE** via `/mission_manager/pause_battery`
    (`pause_reason = BATTERY_CRITICAL`), not STOP.
  - START/RESUME/NEXT/SKIP return 409 `BATTERY_CRITICAL_LOCKOUT`.
  - `cmd_vel_bridge` requests MAVLink `BATTERY_STATUS` (147) at 2 Hz
    (`battery_status_rate_hz`); before this, `/mavros/battery` was ~0.3-0.44 Hz.
  - ✅ Verified 2026-09-30: `/mavros/battery` measured **1.8 Hz, largest gap
    0.51 s**, well inside the 1.5 s freshness limit. If it ever falls back
    to ~0.3 Hz, a critical battery never pauses.
  - ⚠ There is no unlock hysteresis: one sample > 23.80 V clears the lockout,
    and the pack recovers at rest.
- **Power-cycle recovery** (SHARINI `b6eb272`):
  - `/mission_manager/restore_runtime` (`RestoreMissionRuntime.srv`, runtime JSON)
    restores an unfinished mission to PAUSED with
    `pause_reason = POWER_CYCLE_RECOVERY`.
  - The checkpoint is `<runtime_dir>/mission_recovery.json`
    (`settings.mission_recovery_file`), deleted on STOP/CLEAR.
  - Plans: `docs/DYX_Mission_Power_Cycle_Recovery_Logic.md` (SHARINI) and
    `docs/DYX_Persistent_Memory_And_Recovery_Plan.md`.
  - `trajectory_generator`'s `path_signature` hashes **local x/y**, so it changes
    every reboot. Never use it as mission identity across a reboot; use the CSV
    SHA-256.
  - Operator decision: do not raise RTK re-fix offset as a recovery concern.
- **Obstacle monitor fixes** (`3fbc789`):
  - Serial reads block for 1 byte, then drain `in_waiting`.
    `read(64)` had waited the 0.1 s timeout on every frame.
  - Interference frames (0xFFFE) no longer count as sensor health.
  - A reading ≤ `immediate_danger_mm` (800) or the blind zone confirms danger on
    the first frame.
  - `cmd_vel_bridge` sends zero on the danger message itself.
- **RTK / NTRIP facts** (active profile, 2026-09-30):
  - Emlid caster `caster.emlid.com:2101`, mountpoint `MP23960`,
    **GGA disabled**, `direct_inject` to Septentrio USB2.
  - "NTRIP caster connected successfully" followed by `socket_bytes=0` and
    "No first CRC-valid RTCM frame" means the **base feeding the mountpoint is
    not streaming**. It is not a rover fault, and not a missing GGA.
  - When streaming, the rate is ~600 B/s, with 0 CRC errors.
- **Tablet side:** `realtime-hardening` @ `09b3d3a` has all of the above. The
  release APK was built 2026-09-30 from that commit.

### MAVROS abort + slow commands (2026-09-30 evening, not field-tested)

- **MAVROS 2.14.0 (apt) aborts on a duplicate `COMMAND_ACK`.**
  - `command.cpp` delivers an ACK with `promise.set_value()`, keyed only on
    the command number.
  - A second ACK that arrives before the waiter erases its entry throws
    `std::future_error: Promise already satisfied`. Nothing catches it, so
    the process exits with code -6.
  - PX4 ACKs Mission Manager's `DO_SET_MODE` (176) twice here. Two harmless
    `CMD: Unexpected command 176` came first (17:29, 17:32); the crash was
    at 17:34:46, 1 s after a tablet Start.
  - Why PX4 sends two ACKs is not known. Lead: a GCS at MAVLink 255.190 on
    the UDP link.
  - Upstream fixed it after 2.14.0: `97a14ae1` (command, #2159) and
    `6ea0636e` (param, PR #2093). Both are backported in `patches/mavros/`
    and built into `~/mavros_ws` by `scripts/build_patched_mavros.sh`.
  - `rover.launch.py` uses that overlay only if its `mavros_node` exists,
    and logs which MAVROS started.
- **MAVROS now respawns** (`respawn_delay=3.0`). Before this, a dead MAVROS
  left the stack "active" with every Start refused as "Spray controller not
  ready", because the spray controller needs `/mavros/state` < 2.5 s old.
- **Mission Manager `_px4_link_block_reason()`** (stale > 3 s or
  disconnected) blocks START/RESUME and motion health, and is checked
  *before* spray readiness. Status exposes `px4_link_ok` and
  `px4_link_reason`; the tablet shows a `PX4 LINK LOST` toast.
- **Why every command was slow:** `rover_backend` subscribed to
  `/uas1/mavlink_source`, ~350-420 Hz (ATTITUDE ~100 Hz, several 50 Hz
  streams), to decode ESTIMATOR_STATUS, which arrives at < 1 Hz.
  - rclpy costs **~550 us per delivered message on this Jetson, raw or
    deserialized** (measured), so one backend thread sat at ~54% CPU and the
    process at ~100%.
  - API calls and ROS service replies queued behind it on the GIL.
  - Now `estimator_status_relay` (a separate process) forwards only #230 on
    `/rover_backend/estimator_status_mavlink`.
  - Rule: **never subscribe a Python node to `/uas1/mavlink_source`.**
  - Measured after deploy (18:22, patched MAVROS live): backend 83% -> ~70%
    CPU; the relay itself costs ~27%. py-spy `--gil` (sudo needed,
    `ptrace_scope=1`): the backend holds the GIL 67% of a core, and ~70% of
    that is rclpy's `wait_for_ready_callbacks` (the wait set is rebuilt on
    every wake-up), with FastAPI ~12%. **Cost scales with wake-up rate, not
    payload.** The remaining ~140 wakes/s are mostly odom 50 Hz + `/rpp/debug`
    50 Hz.
  - Other load seen: `bag_autorecord.py` polls `/api/mission/status` at 5 Hz;
    MAVROS uses ~94% CPU.

### Field run `mission.csv_20260930_185447` + `log_592` (15 pts) — findings

Bag: `bags_jet/Sep_30_evening/`. The ULog GNSS clock runs **17.1 s ahead** of
the Jetson/QGC clock (aligned on OFFBOARD->ALTCTL); apply the offset before
comparing any bag/ULog timing.

- **"GNSS fusion stopped" is a satellite-count failure, not FLOAT.** The RTK
  solution used only 4-10 SVs all evening, versus 15-16 whenever the receiver
  fell back to 3D fix; RTK counts only SVs in the base corrections
  (`MP23960`).
  - At 4 SVs, `EKF2_REQ_NSATS=6` fails (`gps_check_fail_flags=2`) -> EKF
    dead-reckons -> about 4 s later local pos/vel are invalid -> PX4 sets
    `offboard_control_signal_lost` **with** `local_velocity_invalid` -> ALTCTL.
  - The OFFBOARD stream was intact (0 gaps > 0.2 s). **PX4's "No offboard
    signal" can mean "velocity estimate invalid".**
  - MANUAL/Hold changes the same evening were the RC mode switch (slot 4 then
    back to 1), not the software.
- **radial20 deadlock after stopping inside the 0.75 m terminal zone (FIXED
  in source 2026-09-30, not field-tested).**
  - `_step_radial20_terminal_for_cycle` fed the regulator the *last published*
    speed. After any stop (RTK pause, E-stop release, resume, short leg) that
    is 0, so TRACK/TERMINAL_GUIDANCE passed 0 through, RPP published 0 again,
    and the point failed at `terminal_timeout` after 15 s without moving
    (P0002/9/12 at 111/42/97 mm).
  - Now seeded with `deceleration_floor_speed` (0.15 m/s, above the 0.143 m/s
    breakaway), **only while the chassis is stationary**: TRACK/TERMINAL_GUIDANCE,
    and position-derived speed < 0.02 m/s. It keeps requesting until the
    wheels move, and `publish_velocity_ned` still caps at the approach profile.
    A normal approach (always >= 0.10 m/s) never takes this branch, so the
    **proportional slow approach is unchanged; the operator says that stop is
    correct, do not alter it.**
  - ⚠ **The first version (`f16d7b0`, deployed 19:45) seeded whenever the last
    command was < 0.15.** A normal approach publishes down to 0.10 m/s
    (`approach_speed_cap`, `approach_min_speed_mps: 0.10`), so it altered
    every stop: it raised the braking speed and moved the brake point. The
    operator reported drag/skid at the stop.
  - ⚠ `46e859e` (literal-0 command only) is also wrong. After a stop the
    acceleration ramp restarts at ~0.01 m/s, the seed releases on the next
    cycle, and the rover stalls below breakaway.
  - Fixed in the follow-up commit. Closed-loop tests (regulator + ramp +
    approach cap + chassis breakaway) separate all three versions. **Not
    deployed, not field-tested.**
  - The `/rpp/debug` label `ALIGNMENT` during those windows was misleading:
    the radial20 branch ran and returned first.
- **`/mission_manager/status` was ~97 KB at 4.5 Hz (65% of the bag).**
  - `point_results`, `point_accuracy_snapshots`, `point_survey_snapshots` and
    `last_termination_report` now go on the latched
    `/mission_manager/point_history`, published only on change. Status carries
    `point_history_version` instead.
  - The backend merges history back into the recovery checkpoint
    (`_with_point_history`), and only for the same `mission_run_id`.
  - The bag recorder topic list and QoS overrides include the new topic.

### Base corrections vs receiver use (2026-09-30, receiver fw 4.14.0)

- **What arrives:** `DiffCorrIn` on a temporary USB1 stream (parked, 5 min,
  streams turned off afterwards) shows the base sends 1005 + MSM4 GPS 9 /
  GLO 6-7 / GAL 7 / BDS 7 sats + 1230, all at 1 Hz, 0 CRC errors, max gap
  1.2 s. The base is about 6 m away.
- **Format is fine (checked 2026-10-01 from the same capture):** standard
  MSM4, station ID 0 in every message, correct multiple-message chaining (GPS/
  GLO/GAL flagged "more follow", BDS last), and BDS epoch times are in BDT
  (they would read 14.9 s off if the base had used GPS time; they read 0.9 s).
  Messages arrive 0.5-0.9 s after their epoch.
- **Per-satellite signal content (the signal MASK hides this):**
  - GAL: all 7 sats carry E1+E5bQ, every epoch.
  - GPS: 6-7 of 9 carry L1+L2C.
  - **BDS: only C10 has B1I+B2I; the other 7 (C07, C08, C13, C23, C28, C32,
    C37) are B1I only.** BDS is almost entirely single-frequency from this
    base. No GEO satellites are sent.
  - ⚠ The earlier bullet "signals match the rover exactly (B1I+B2I)" compared
    the mask, not the satellites, and was wrong in effect.
- **What the receiver used:** BDS 0%, GAL dropped in 10% of epochs, NrSV 6-9,
  `RxError=8` throughout. Why BDS is unused is **not established**: a normal
  dual-frequency requirement on thin BDS content would explain it, but that is
  an untested assumption about Septentrio's RTK engine. The base sent GAL
  steadily while the receiver dropped it for ~3.5 s stretches, so the GAL
  drops point at the rover side (tracking/ambiguity), not the base.
  I have **no per-satellite rover tracking data** (SBF ChannelStatus 4013) for
  that capture, so "rejected" and "not usable" cannot be told apart yet.
- **Mission ULog (`gps_dump`):** every FLOAT window is exactly the time
  corrections-available shrank to GPS-only (97 s, 4 SVs) or GPS+GAL, while
  correction age stayed 1.1-2.5 s. The 0.5 s StandAlone/DGPS epochs used
  15-16 SVs across all four constellations, so sky and tracking were fine.
- **PX4 is not the cause:** `vehicle_gps_position` matches the receiver
  `PVTGeodetic` in 99.9% of epochs (fix and NrSV).
- ⚠ **PX4 `time_utc_usec` from this driver is GPS time** (18 s ahead of UTC),
  which is the ULog clock offset.
- Streams 2-10 on the receiver are all off now (USB1 is silent unless a
  stream is configured).
- Base distance: the baseline was **13.68 km on 2026-09-08/09 and is 6 m now**,
  so this is a different base. BDS "working on Sep-08" is not evidence of a
  regression here.
- Firmware: the 4.14.10 "NTRIP burst" note we quoted concerns the receiver's
  built-in NTRIP client. This rover pushes RTCM over USB2 and NTR1 shows
  "waiting", so it may not apply. Not read from Septentrio's notes.
- **The base is a ZED-F9P on a Raspberry Pi** (repo
  `/Users/dyx_a1/Vetri/gnss_fastapi_backend`, Pi `192.168.1.100`, pushes NTRIP
  over eth0 to `caster.emlid.com`). Read from its source 2026-10-01:
  `save_all_config.py` enables GPS/GLO/GAL/BDS with BDS B1 **and** B2; live
  RTCM is **MSM4** (matches our capture; that repo's docs and autoflow default
  say MSM7, so they are stale). Its journald is **persistent** and logs a
  push line every 5 s (frames, bytes), so the base side CAN be checked after
  the fact: `ssh dyx@192.168.1.100` (not `.101`), `journalctl -b -1 -u
  gnss-backend`. It does not log per-satellite content.
- **Hypothesis, NOT verified against u-blox/BeiDou documents:** a ZED-F9P
  tracks only BDS B1I/B2I, and BDS-3 satellites carry no B2I (they use
  B2a/B2b), so most BDS from this base is B1I-only. That fits the capture:
  4 of 4 BDS-3 MEO sats (C23/28/32/37) and 3 of 4 BDS-2 IGSO sats (C07/08/13)
  are B1I-only, and only C10 is dual. MSM7 would not add signals.
- **BDS is probably NOT the cause of the FLOAT drops.** BDS was unused in the
  good FIXED epochs too (NrSV 8-12). The drops are GAL/GLO going missing.
  Open candidates: rover-side tracking (obstruction, multipath, or dusk
  scintillation at 13 N in equinox season; all trouble was 18:55-19:02 and
  20:03 IST; untested) versus the base side.
- **Base side is EXCLUDED for the 97 s GPS-only window (measured 2026-10-01
  from the base journal).** Epoch size was identical before, during and after
  the drop: 581 B (18:55-18:56:25 FIXED), 581 B (18:56:29-18:58:06, 4 SVs),
  580 B (18:58:20-19:03), 580 B (my 20:03 capture); 6 frames/s throughout.
  The base kept sending the same satellites. So the loss is downstream
  (caster -> Jetson RTK worker -> USB2 -> receiver) or rover-side tracking.
- **Other base facts (2026-10-01):**
  - The push dropped at 18:28:56-18:29:32 (3 attempts, "NTRIP 1.0 rejected",
    216 queued frames dropped): a ~36 s outage before the 18:54 run. The base
    pushes with **NTRIP 1.0**.
  - The base is in **FIXED mode on a saved position surveyed 2026-06-22 with
    `accuracy` 0.92 m** (survey limits: 2.0 m, 15 s). It is stable across
    reboots (no re-survey), but **absolute** coordinates are good to about 1 m;
    this matters only if mission points were not surveyed against this same
    base.
  - The Pi's root disk is **95% full (1.5 GB free)**; it hit 0 on 2026-09-09.
- **Discriminators now:** (1) the Jetson's own RTK worker log for
  18:56:29-18:58:06 (its per-5 s frames/bytes/crc lines in `journalctl -u
  dyx-rover`) to see whether the corrections reached the Jetson in full;
  (2) the same-base daytime bundles of 2026-09-30 16:50-17:13 (fix % and NrSV
  versus dusk); (3) SBF ChannelStatus (4013) on the temporary USB1 stream: per
  satellite, tracked / used in PVT, with a reason.

## Repo status (2026-08-31)

- **`WORKING_STATUS_24_06_2026.txt`** — stale (2026-06-24), architecture it
  describes no longer exists. Historical context only.
- **`audit_report.md`** — current (dated today). RTK code review "NEEDS
  FIX" (§ above); separately confirms the primary consumer of
  `rover_backend`'s API is the verified React Native rover app in the GCS
  section below.
- **`REFACTOR_TO_CPP_AUDIT.md`** — current (dated today, branch
  `feat/rtk-injection-v2`). 9 `ament_python` packages, zero C++, ~50,650
  lines across 55 production files. **104 dead `.before_*`/`.bak_*` backup
  files (~5.1MB, mostly in `rpp_controller`)** flagged for deletion
  regardless of any refactor decision. `rpp_controller_node.py` is a single
  **9,281-line class**, rated hardest to port (9.5/10) and 55-65% of a
  hypothetical C++ port's cost. Overall system scored **5.5/10 —
  "CONDITIONAL PRODUCTION: safe only with an operator in the loop"**;
  weakest categories control-loop determinism (GIL, 4/10) and code hygiene
  (dead files, 3/10).
- Stray zero-byte files at repo root (`0.0`, `=`, `Function`, `P1`, `P2`,
  `lower`, `self.closest_marking_distance`, and similar) look like accidental
  shell-redirection artifacts, not meaningful content — safe to ignore, low
  priority to clean up.
- `.before_*_<timestamp>` / `.bak_*` files litter `src/` — **always read the
  live file without the suffix**; a stale sibling next to the real node
  source is the normal state in this repo, not a sign something's wrong.

## GCS frontend — verified

The verified 4WD GCS frontend is
`/Users/dyx_a1/Vetri/temp/DYX_GCS_Frontend` (package `dyx-gcs-mobile`,
app id `com.dyxgcs.mobile`, Expo 54 / React Native 0.81). The working branch
is `realtime-hardening`, and `main` is fast-forwarded to it. The rover is
`192.168.3.101:5001`. The backend is this `rover_ws` workspace, under
`src/rover_backend`, not the separate `/Users/dyx_a1/Vetri/4WD_SERVER`
repository.

- **How the app finds the rover** (corrected 2026-09-30): `src/config.ts` has
  **no hard-coded rover IP**. The address comes from:
  1. the saved selection (`utils/backendStorage.ts`, port default 5001);
  2. beacon discovery (`utils/autoConnect.ts` + `services/beaconListener`,
     backend `DYX_BEACON_*`);
  3. manual entry.

  `EXPO_PUBLIC_ROS_HTTP_BASE` / `EXPO_PUBLIC_ROS_WS_URL` in `.env` are not read
  by `src/`. They were `192.168.1.101` (old router) and are now corrected to
  `.3.101`, but changing them does nothing.
- **Socket events the tablet consumes** from this backend:
  - `telemetry` (incl. `battery_*`, `obstacle`)
  - `mission_status`
  - `point_*`
  - `trajectory_path`
  - `rtk_status` (2026-09-30; REST `/api/rtk/status` is only a fallback)
  - `system_event` (2026-09-30; toast in `SystemEventNotice`)
  - recovery fields in mission status (`MissionRecoveryNotice`, SHARINI)
- **Release APK (local, verified 2026-09-30):**
  - `android/` is generated by `expo prebuild` and gitignored.
  - Release signing reads `DYX_RELEASE_STORE_FILE/_STORE_PASSWORD/_KEY_ALIAS/_KEY_PASSWORD`
    from `~/.gradle/gradle.properties`; the build **fails** without them. It never
    falls back to the debug key.
  - Always `npm ci` first. `node_modules` drifted from the lockfile once
    (axios 1.18.1 vs 1.20.0).
  - Then `cd android && ANDROID_HOME=~/Library/Android/sdk ./gradlew assembleRelease`
    → `android/app/build/outputs/apk/release/app-release.apk` (~72 MB).
  - Verify the signer with `apksigner verify --print-certs`. The release certificate
    is `CN=Vetrivelan Velmurugan, OU=DYX_GCS`, SHA-256 `5eaacb96…4c9a`.
  - The version is still 1.0.0 / versionCode 1, so bump `app.json` if tablets must
    see an upgrade.
  - Install with `adb install -r`.

## Hard rules

- Do not run `scripts/start_rover.sh` as a restart mechanism — it's broken
  (see above). Use `rover.launch.py`, or ask the operator how they actually
  bring the stack up.
- Do not assume RC-loss / datalink-loss disarms the rover — `NAV_RCL_ACT`/
  `NAV_DLL_ACT` are not set to Disarm here, unlike the 3WD sibling.
- Do not confuse this backend with `/Users/dyx_a1/Vetri/4WD_SERVER`; that is
  a different repo. The verified frontend is
  `/Users/dyx_a1/Vetri/temp/DYX_GCS_Frontend`.
- Do not trust a static `.params` export in `px4-params/` over a
  same-session live param capture from a bag bundle — the exports disagree
  with each other on at least one FCU param.
- ArduRover is abandoned — do not propose ArduRover solutions (carried over
  from the 3WD project's rule, applies equally here: this is a PX4 rover).
- **The PX4 ulog is authoritative for FCU params, above any QGC export.** On
  2026-09-01 an export read `RO_SPEED_LIM=0.4` while the ulog for the same
  runs read `1.2`. ulogs embed the full param set plus firmware version and
  git hash — read them with `pyulog`.
- **PX4 ulog FILENAME times are unreliable.** Reconstruct wall time from the
  embedded GNSS UTC and confirm the bag<->ulog pairing by cross-correlating a
  shared signal (yaw rate works; expect the NED/FRD <-> ENU/FLU sign flip).
- **Never judge "is the rover stopped/how fast is it" from the EKF velocity**
  (`twist.linear` on `/mavros/local_position/odom`). It rings through zero for
  seconds after a pivot. Differentiate POSITION over a ~0.5 s window instead,
  **and gate on yaw rate too** (position-derived speed cannot tell parked from
  pivoting in place), and say which one you used for every number you report.
  ⚠ The "~0.023 m/s stationary noise floor" quoted elsewhere in this file is
  the post-pivot RINGING, not the floor: gated on actuator output + gyro with
  a settle trim, the genuinely parked floor is **6.3 mm/s 2D RMS** (2026-09-05,
  §Task 2.1). The 0.030 -> 0.060 pivot-settle gate was sized against the
  ringing and stays correct.
- **`RD_MAX_THR_YAW_R` is a wheel-speed difference in m/s, not a yaw rate.**
  Full-differential yaw rate is `2 * RD_MAX_THR_YAW_R / RD_WHEEL_TRACK`.
- **Do not hand-roll CDR parsing for bags.** Import the verified `_CDR` class
  and parsers from `scripts/analyze_mission.py`. A wrong-alignment parse
  yields finite-but-absurd values that still plot — this already happened
  once here.
- **Never compare positions across an RTK re-fix.** A `fix_type` drop below 6 and
  return can move the reported position by 100-500 mm with no change in `eph`,
  satellite count or DOP (2026-09-09: 192 mm in `log_15`, -210 mm in `log_18`).
  Split every dataset on re-fix boundaries before averaging, differencing, or
  calling anything a bias.
- **Never read a mission-log stop error as GNSS accuracy.** The rover closes the
  loop on this same GNSS and stops when its *reported* position reaches the
  surveyed target, so a receiver bias is absorbed into the physical parking
  position and does not appear in the reported error. Mission-stop numbers
  measure controller convergence. Use an open-loop channel instead — **height**
  is the one the rover never controls — or a static placement test.
- **Do not hand-roll SBF or RTCM parsing either.** Use
  `scripts/gps_dump_sbf_report.py`; its layouts are size-checked against the
  receiver's own `Length`/`SBLength` at decode time. ArduPilot's
  `VectorInfoGeod.ReferenceID` is `u1` and is wrong — spec and receiver say `u2`.
- **PX4 does not configure the Septentrio antenna or its constellation set.**
  No antenna/ARP/PCV command exists in the driver, and `SEP_CONST_USAGE = 0`
  suppresses `ssu`. Both are receiver NVRAM and survive — or change across — a
  power cycle, in either `SEP_AUTO_CONFIG` state. Do not infer receiver
  configuration from PX4 parameters.

## Quick reference

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/odom
ros2 topic echo /rpp/accuracy --once
journalctl -u bag-autorecord.service -f      # bag recorder unit
journalctl -u dyx-rover -f                    # rover stack unit
journalctl -u dyx-rover -b | grep "MAVROS:"   # patched overlay vs apt MAVROS
sudo systemctl restart dyx-rover bag-autorecord   # restart (rover idle + disarmed first)
iw dev wlP1p1s0 station dump | grep -E "^Station|signal:"   # hotspot clients
sudo iptables -t mangle -L DYX_HOTSPOT_QOS -n -v            # client-internet cap counters
```

- Bag bundles: `bags_jet/<mission.csv_timestamp>/` (sometimes nested under
  `bags_jet/<DD_MM_YYYY>/Patch_NN/`) — see the `analyse-missions` skill for
  full anatomy and a working decoder (`scripts/analyze_mission.py`).
- `python3 scripts/pivot_settle_report.py <bundle-dir> [--threshold 0.060]`
  — per-pivot dwell breakdown: when rotation physically stopped vs when the
  stationary gate cleared, i.e. how long was spent waiting on the estimator
  rather than the rover. Also prints the gate signal's stationary noise floor,
  judged from position so the measurement does not assume its own conclusion.
- `python3 scripts/gps_dump_sbf_report.py [--reference targets.csv] <*.ulg>`
  — decodes the raw Septentrio SBF and injected RTCM3 out of `gps_dump`: which
  signals/constellations the RTK solution used, ARP/phase-centre flags, base
  correction contents, receiver uptime, and stationary error vs surveyed truth.
  ⚠ With `direct_inject` (USB2) the ULog carries **no** RTCM (the `.rtcm`
  stream is 0 bytes). Only the receiver's own `BaseVectorGeod` "corrections
  available" signal mask is in the log.
- **Receiver console (USB1 = `/dev/ttyACM1`, free; USB2 belongs to the RTCM
  worker).** Open it raw and send ASCII commands ending in `\r\n`.
  - Read-only: `getSBFOutput, all`, `getDataInOut, all`, `getSignalUsage`,
    `getSatelliteUsage`, `getDiffCorrUsage`, `lstInternalFile, Identification`.
  - Do not send `lif, error` without asking: it **clears** the error log.
  - To see raw corrections, set `Stream2 -> USB1, DiffCorrIn, OnChange` plus
    `Stream3 -> PVTGeodetic+BaseVectorGeod+ReceiverStatus`, capture, then set
    both back to `none, off` and re-read to confirm. Never save to boot: the
    receiver config is user-owned (memory: gnss-manual-config-architecture).
    Decode the RTCM inside SBF 5919 with `scripts/gps_dump_sbf_report.py`
    helpers; MSM signal mask = DF395 at bit 137.
- `python3 scripts/nsh_bridge.py "<cmd>"` on the Jetson: PX4 NSH over MAVROS
  (`commander check`, `listener failsafe_flags -n 1`, ...). It refuses while
  armed.
- Backend: `http://192.168.3.101:5001` (FastAPI, port from
  `src/rover_backend/config/backend.env`).
- Deploy/review workflow: see the `rover-ship` skill. Deploy is git only:
  commit, push, then `git pull` on the Jetson, then `colcon build` (only when
  entry points, `.srv` files or `package.xml` change), then
  `sudo systemctl restart dyx-rover`. Check first that the rover is idle and
  disarmed (`/mission_manager/status`).
