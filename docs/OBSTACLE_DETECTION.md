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
| **unknown** (no valid frame for 0.5 s) | **Warning only:** system event `OBSTACLE_SENSOR_LOST` and voice "Obstacle sensor lost" once during a mission. The mission **continues without obstacle protection**. |

**Details:**
- A zone is confirmed after **3 consecutive readings**. Leaving danger also
  needs the object to be more than **200 mm** past the danger distance.
- The sensor's "no echo" reading means a clear path. The blind-zone reading
  (0) counts as **danger**. Interference frames keep the sensor "alive" but
  don't change the zone.
- **Active only while RUNNING.** At a point (`WAITING_FOR_NEXT`, e.g. the laser
  operator standing there) it doesn't pause, but NEXT is refused while someone
  is within the danger distance.
- **Why 1.5 m:** at 1 m/s, debounce plus reaction is about 0.4 m, and braking
  adds 0.3–0.5 m.

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
- **Detection keeps running while paused,** so Resume is offered only once the
  path is clear.
- **Independent second layer** in `cmd_vel_bridge`.
- **Status is published immediately on a zone change,** and mission_manager
  reacts in the callback, not on its next tick.
