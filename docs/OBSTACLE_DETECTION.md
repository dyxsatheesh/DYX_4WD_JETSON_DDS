# Forward obstacle detection (stop and wait)

Added 2026-09-29. Ported from the 3WD NRP_ROS (`Backend/obstacle_monitor.py`).
It is unit-tested only; **the sensor is not fitted on the 4WD yet**.

**Next feature:** avoidance, i.e. driving around an obstacle. It is not part of
this.

## Operator decisions (2026-09-29)

| Question | Decision |
|---|---|
| Hardware | Same as NRP_ROS: **DYP A21 (A2111BU) ultrasonic, UART**, forward |
| Obstacle in the danger zone | **Pause the mission; manual resume only** |
| Sensor failure while enabled | **Warning only**; the mission continues |
| Behaviour | **Stop and wait** |

## Behaviour

| Zone (default) | Rover |
|---|---|
| **danger** ≤ 1.5 m | RUNNING mission → **PAUSED, reason `OBSTACLE`**. Motion off at once. Voice: "Obstacle. Mission paused." START / RESUME / NEXT are refused while danger persists. Resume is offered ("Path clear…") **only after the path clears**. It never auto-resumes. |
| **warning** ≤ 3.0 m | Voice "Obstacle ahead, N centimetres" (while RUNNING, 10 s cooldown). No stop. |
| clear | nothing |
| **unknown** (no usable reading for 0.5 s) | **Warning only:** system event `OBSTACLE_SENSOR_LOST` and voice "Obstacle sensor lost" once during a mission. The mission **continues without obstacle protection**. |

**Details:**
- A zone is confirmed after **3 consecutive readings**, except that a
  reading at or inside **0.8 m** (`immediate_danger_mm`), or the blind-zone
  reading, confirms **danger on the first frame**. Leaving danger is always
  debounced and also needs the object to be more than **200 mm** past the
  danger distance.
- The sensor's "no echo" reading means a clear path. The blind-zone reading
  (0) counts as **danger**. Interference frames (`0xFFFE`) change nothing and
  do **not** keep the sensor healthy: a sensor reporting only interference is
  blind and goes `unknown` after 0.5 s. They are counted in
  `interference_frames` in the status.
- **Transport:** the reader blocks for one byte and then takes whatever is
  buffered, so a frame is handled as soon as it arrives (it used to wait up to
  0.1 s for a 64-byte read). A zone change is published at once, and
  `cmd_vel_bridge` sends zero on the danger message itself rather than on its
  next 100 Hz check.
- **Active only while RUNNING.** At a point (`WAITING_FOR_NEXT`, e.g. the laser
  operator standing there) it doesn't pause, but NEXT is refused while someone
  is within the danger distance.
- **Why 1.5 m:** at 1 m/s, debounce plus reaction is about 0.25 m (estimated
  from code, ~10 Hz sensor; not yet measured), and braking adds 0.3–0.5 m.

## Architecture

```
A21 ──UART──► obstacle_monitor ──/obstacle/status (JSON, 10 Hz + on change)──┐
                                                                              ├─► mission_manager: pause OBSTACLE / gate START·RESUME·NEXT
                                                                              ├─► cmd_vel_bridge: 2nd layer, zero velocity on danger
                                                                              └─► rover_backend: /api/obstacle, tablet state, voice
```

**`obstacle_monitor`** (new ament_python package):

| File | Role |
|---|---|
| `a21_protocol.py` | frame parser (pure) |
| `zones.py` | debounce, hysteresis, health (pure) |
| `obstacle_monitor_node.py` | serial thread with reconnect, status, config |

The node **never commands motion**. Settings are in
`~/.ros/dyx_obstacle_settings.json`, which overrides the launch values once
saved:
- `enabled` (default **false**)
- `danger_mm`, `warning_mm`
- `port` (default `/dev/ttyTHS1`), `baud` (115200 / 9600)

**`mission_manager`:**
- `_monitor_runtime_obstacle()`, run on every status message and every
  control tick;
- `_obstacle_danger_distance_mm()` inside `_motion_health_status`;
- pause reason `OBSTACLE`;
- `obstacle` block in the mission status.

**`cmd_vel_bridge`:** `obstacle_gate.obstacle_blocks_motion()`. Reason
`obstacle_danger` sits right after `emergency_stop`.

**Both use the same contract:** only a **fresh (≤ 1 s), healthy, enabled
danger** reading stops the rover.

## Hardware path (decided 2026-09-29)

**Now: the current UART A21, through a USB hub.** The Holybro Pixhawk Jetson
baseboard has no 40-pin header and all 4 USB ports are in use (LoRa, GNSS,
USB speaker, 4G modem).
- Move **LoRa** and a **CP2102 / FTDI USB-UART adapter** onto a small USB 2.0
  hub. Keep the 4G modem and GNSS on direct ports.
- Don't use a CH340 adapter: the RTK worker's CH340 has no serial number, so
  two of them get the same by-id name.
- The setup script saves the adapter's `/dev/serial/by-id/...` path.

**Future: the A21 I2C version** on the Jetson I2C connector. The A21 family
has **no SPI version**. Its outputs are UART auto / controlled, PWM, switch,
RS485 (Modbus), I2C and CAN, fixed per model at purchase.
- **Addressing:** 20 selectable I2C addresses (default 0xE8 = 7-bit 0x74; set
  one sensor at a time through register 0x05).
- **Triggered readings:** the host triggers each reading (register 0x10 ←
  0xB0 for 5 m range, then read register 0x02), so several sensors can fire in
  turn without cross-talk. That suits the avoidance feature.
- **Electrical:** logic level = supply, so power at 3.3 V. 3–10 kΩ pull-ups;
  10–100 kbit/s; keep the wiring short and away from the motor-controller
  cables.
- **Code:** `obstacle_monitor` needs an I2C reader; the zones, pause, bridge
  and API stay the same.
- **Long cable runs:** if I2C proves noisy, the RS485 / Modbus version on one
  USB-RS485 adapter is the robust fallback.

**Deliberately not used:** the PX4 / Pixhawk route. PX4 has no A21 driver,
and the A21 CAN version uses DYP's own protocol, not DroneCAN.

## Setup (per rover)

**Wiring:** sensor VCC → 5 V, GND → GND, **sensor TX → Jetson UART RX**. On
the Orin Nano 40-pin header that is pin 10 = `/dev/ttyTHS1`. Check the A21 TX
logic level against the Jetson's 3.3 V input.

```bash
ssh flash@192.168.3.101 "cd ~/rover_ws && ./scripts/setup_obstacle_sensor.sh"   # add --enable to switch it on
```

The script:
- runs `colcon build` for the new package (a **new package needs a build
  once**);
- checks `dialout` and pyserial;
- listens on each free UART at both baud rates for A21 frames;
- saves the port it finds.

Restart the stack afterwards.

## Topics and verification

| Topic | Type | Use |
|---|---|---|
| `/obstacle/status` | `std_msgs/String` JSON (`dyx4wd/obstacle@1`), 10 Hz + on zone change | **Safety contract** read by mission_manager, cmd_vel_bridge and the backend |
| `/obstacle/range` | `sensor_msgs/Range` (ULTRASOUND, frame `obstacle_front`, 0.03–5 m, 70° FOV), every reading | Standard ROS view for RViz / plotting / future avoidance. REP 117: `+inf` = nothing in range, `-inf` = blind zone |
| `/obstacle/config` | `std_msgs/String` JSON with `request_id` | Settings changes (the backend sends these) |

All three are recorded in the mission bags (`scripts/bag_autorecord.py`). The
node's parameters are saved with each bag.

**End-to-end check, safe while the stack runs:**
`python3 scripts/test_obstacle_pipeline.py` (with ROS sourced).
- It starts a separate test node on a virtual serial port.
- It feeds real A21 frames through clear → warning → danger → silent →
  recover.
- It checks the zone, health, distance, the Range values and the status rate.

## API (login required)

| Endpoint | Body |
|---|---|
| `GET /api/obstacle/status` | `status`: enabled, healthy, zone, distance_mm, danger_mm, warning_mm, port, port_open, serial_error, readings, bad_checksums |
| `POST /api/obstacle/config` | any of `{"enabled", "danger_mm", "warning_mm", "port", "baud"}`; needs 300 ≤ danger < warning ≤ 7500; 409 with the reason if rejected |

The tablet's existing `setObstacleDetection` / `updateObstacleZones` calls map
onto `/api/obstacle/config` (NRP_ROS `danger_max_mm` / `warning_max_mm` →
`danger_mm` / `warning_mm`).

**Frontend still to do:**
- load the toggle state from `GET /api/obstacle/status` (today it starts at
  off);
- show distance and zone;
- show pause reason `OBSTACLE`, and the error message from `ApiError`.

## Fixes vs NRP_ROS

- **Zone settings actually apply.** NRP_ROS stored the UI zones but used
  hard-coded 2000 / 4000 mm.
- **A dead sensor is reported.** NRP_ROS silently gave no protection.
- **A silent port is reopened.** On 2026-09-29, in the first mission with
  detection on, the FT232 read path stalled in the kernel
  (`ftdi_sio ... urb stopped: -32`) about 80 s after the rover started
  driving. The port stayed open with no data and no error, so the sensor
  was lost for the rest of the run. `obstacle_monitor` now closes and
  reopens a port that delivers nothing for 2 s (`SILENCE_REOPEN_SEC`),
  retrying until data returns, with `serial_error` = "no data from ...". The
  likely root cause is motor EMI or power on the USB hub: keep the adapter on
  a direct Jetson port or a powered hub, away from the motor-controller wiring.
- **Detection keeps running while paused,** so Resume is offered only once the
  path is clear.
- **Independent second layer** in `cmd_vel_bridge`.
- **Status is published immediately on a zone change,** and mission_manager
  reacts in the callback, not on its next tick.
