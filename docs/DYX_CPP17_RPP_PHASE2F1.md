# DYX Autonomous 4WD — Phase 2F1: PX4 DDS shadow telemetry adapter

**Starting branch:** `Jetson_DDS_Upgrade` at commit `4aacc9a`.
**Scope:** Additional ROS 2 Humble node for receive-only PX4 DDS telemetry and *local diagnostics*. **No motor output, no reverse movement, no change to working Python RPP, no change to PX4 firmware or rover launch.**

## Design

* `dyx_rpp_cpp17_shadow` subscribes to `/fmu/out/vehicle_local_position` and `/fmu/out/vehicle_angular_velocity` using sensor-data QoS.
* Publishes **only** `/dyx/rpp_cpp17_shadow/status` (`std_msgs/String`). It never publishes `/fmu/in/*`, `/rpp/command`, `/mission_enable`, arm/disarm, RC, or servo outputs.
* `shadow_enabled=false` by default; even when explicitly true, all computed velocities/yaw rates are **diagnostic previews** only. The synthetic goal is configured in PX4's local EKF origin and is not a mission waypoint, GNSS marking location, or valid trigger for spray.
* Checks `xy_valid`, `v_xy_valid`, `heading_good_for_control`, finite measurements, PX4 `timestamp_sample` skew, and host steady-clock receive freshness. A position or heading reset sets a restart-required inhibit. After a preview starts, telemetry failure latches the *internal* yaw planner's safe hold. No actuator publication is possible in any state.
* Converts PX4 local North/East to ENU East/North and heading `yaw_enu=wrap(pi/2 - heading_ned)`. Uses `-VehicleAngularVelocity.xyz[2]` only as a **near-level yaw-rate approximation**; FRD body Z is not generally equal to earth yaw rate at nonzero roll/pitch. A later phase must use fully time-aligned attitude before any actuation.
* Uses Phase 2E3 `ForwardYawRatePlanner` only for non-actuating preview; `pivot_permission=false` means it will not preview a permitted pivot maneuver. Reverse-driving code is deliberately absent.
* Does **not** yet integrate mission/trajectory upload, path projection, longitudinal regulation, radial-stop certification, marking, RTK FIX/FLOAT policy, battery safety or spray. These remain in the existing Python rover until separately ported and validated.

## Windows native test (no ROS2 required)

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native -B build/dyx_rpp_native -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_rpp_native --config Release
ctest --test-dir build/dyx_rpp_native -C Release --output-on-failure
```

Expected **22 tests** (19 existing plus `shadow_geometry_native_smoke`, `shadow_geometry_contract`, and `shadow_node_non_actuating_source_contract`). Passing native tests does **not** validate the ROS2 compilation.

## Jetson ROS2 verification (NO rover movement)

The ROS2 package requires PX4-v1.17-compatible `px4_msgs` and your existing Humble workspace. Build and run on the Jetson only after checking for duplicate active bridge nodes:

```bash
cd ~/DYX_4WD_JETSON_DDS
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to dyx_rpp_cpp17_shadow
source install/setup.bash
ros2 run dyx_rpp_cpp17_shadow dyx_rpp_cpp17_shadow_node
```

The default output is `DISABLED`. To **only** preview telemetry in a separate terminal:

```bash
ros2 run dyx_rpp_cpp17_shadow dyx_rpp_cpp17_shadow_node --ros-args -p shadow_enabled:=true -p preview_speed_mps:=0.0
ros2 topic echo /dyx/rpp_cpp17_shadow/status
```

This node does not require the vehicle to be armed or placed in Offboard. Keep motors disarmed. **Do not enable the motion DDS bridge** `dyx_px4_dds_bridge` at this phase.

## Release gates

1. 22/22 Windows native tests.
2. Jetson ROS2 colcon build against installed compatible `px4_msgs`.
3. Runtime inspect DDS topic types, rates and output status while disarmed.
4. Validate yaw ENU/NED conversion at known rover headings and time synchronization; verify `heading_good_for_control` behavior and local position reset handling.
5. Only in later phases implement a genuine trajectory/RPP-to-DDS command adapter with independent safety and field tests.

**Do not infer real-world 20 mm marking accuracy from estimator-local diagnostics.**
