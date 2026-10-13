# DYX 4WD C++17 — Phase 2E3: Forward-only yaw-rate planner

**Base commit:** `e3cd0d7` (`Jetson_DDS_Upgrade`). **Authoring status:** offline native tests only; no Jetson/PX4 hardware authorization.

## Scope

New standalone header `forward_yaw_rate.hpp`: signed ENU yaw-rate steering, Jetson-authorized stationary pivot, safe hold, no negative linear speed. No ROS 2 publisher or firmware changes. All reverse-driving planners are **explicitly excluded**; do not apply the discarded Phase 2E2 patch.

### Confirmed firmware contract (DYX PX4 v1.17 branch `PX4_DDS_Upgrade`, at `2fafa6a`)

`DifferentialOffboardMode::offboardControl()` combined mode `offboard_control_mode.velocity=true, body_rate=true, position=false, attitude=false` uses:

* `speed_body_x = trajectory.velocity[0] * cos(current_yaw_NED) + trajectory.velocity[1] * sin(current_yaw_NED)`
* `yaw_rate_setpoint = trajectory.yawspeed` (NED rad/s)

Negative body-X speed is possible if a target-facing velocity vector opposes current heading. Therefore this planner constructs velocity from the **measured current heading**, not the desired target heading: `north=forward*sin(yaw_ENU)`, `east=forward*cos(yaw_ENU)`. The projected body-X is nonnegative by construction. Desired heading correction is carried *independently* by signed yaw rate. ENU yaw-rate is positive CCW; NED yaw-rate is the opposite sign.

### Planner safety

* External gate `motion_enabled`, fresh telemetry, finite data and nonnegative speed. Any invalidity latches `SAFE_HOLD` until explicit `reset()`.
* For heading error >=15°, no forward translation: `WAIT_PIVOT` until external pivot permission; on permission enter stationary `PIVOT`.
* Pivot holds zero translation until heading error <=2° **and** measured yaw rate <=0.05 rad/s continuously for 0.20 s; pivot timeout is 10 s.
* Pivot target changing >0.5° during the maneuver latches a safety stop.
* For errors below pivot threshold, forward tracking uses yaw proportional control (`Kp 0.85`) capped at 0.15 rad/s, with 0.5° deadband.
* Pivot yaw command capped at 0.18 rad/s; wheel rotations will be opposing during physical differential spot turns. This is **not** reverse-driving the chassis.
* `stop_requested` commands zero velocity and zero yaw rate. `pivot_complete` releases with zero-speed on that tick; forward motion only on the following tick.

### Explicit exclusions / outstanding gates

This is a **new design**, not a port of the old Python pivot calculations; the contract test is *not* Python/C++ parity. Neither Phase 2E2 reverse code nor any ROS2 actuator publishing has been enabled. PX4 must be built/flashed and verified in bench tests; the current DDS bridge must be safety reviewed before this planner is integrated. Position/yaw inputs must be fresh, time-aligned, and use calibrated antenna-to-body yaw/lever arm. Verify that `vehicle_attitude` NED heading and Septentrio heading align, and account for rear GNSS antenna arc displacement in spot turns. No 20 mm real-world accuracy is claimed.

On Windows run `cmake --build build/dyx_rpp_native --config Release`, then `ctest --test-dir build/dyx_rpp_native -C Release --output-on-failure`. Expected **19** tests (existing 17 + two new forward-only tests). Only commit if all pass.
