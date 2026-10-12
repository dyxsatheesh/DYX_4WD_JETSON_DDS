# DYX 4WD — C++17 / PX4 1.17 DDS Phase 1

**Target branches:** Jetson: `Jetson_DDS_Upgrade`; PX4: `PX4_DDS_Upgrade`.
**Status:** Opt-in test bridge only. Not a full Python-to-C++ conversion and NOT production approved.

## Verified from the custom PX4 firmware

- `boards/px4/fmu-v6x/default.px4board` enables `CONFIG_MODULES_UXRCE_DDS_CLIENT=y`.
- `src/modules/uxrce_dds_client/dds_topics.yaml` exposes `/fmu/in/offboard_control_mode`,
  `/fmu/in/trajectory_setpoint`, and `/fmu/out/vehicle_status`.
- `DifferentialOffboardMode.cpp` has a **custom combined mode**:
  `offboard_control_mode.velocity=true`, `body_rate=true`,
  `position=false`, `attitude=false`. It projects NED velocity onto the
  rover's current heading (signed body-X speed) and passes through
  `trajectory_setpoint.yawspeed` to the rate loop.
- `RoverDifferential.cpp` has a 500 ms stale-command stop latch that
  resets only when **disarmed AND outside Offboard**.

**Do not substitute** direct `RoverSpeedSetpoint` and `RoverRateSetpoint`
publishers for the combined mode without changing/verifying the custom
PX4 watchdog contract. Although those topics exist in DDS, the custom
path explicitly checks `OffboardControlMode` and `TrajectorySetpoint`.

## Patch contents

New ROS 2 Humble `ament_cmake` package `dyx_px4_dds_bridge`:
- Reads existing `/rpp/command` (`rpp_interfaces/msg/RppCommand`).
- Converts North/East velocity directly to NED North/East.
- Negates ENU yaw rate to obtain NED yaw rate (CCW to CW sign).
- Publishes 50 Hz combined mode and trajectory setpoints.
- Gates motion on: explicit startup parameter, emergency stop, mission
  enable, backend heartbeat, fresh PX4 armed/Offboard status, valid
  fresh RPP command.
- Defaults to `enable_motion=false` and is **NOT auto-launched**.

## Not yet implemented (required before field deployment)

- Obstacle direct inhibit in the DDS bridge; current Python
  `obstacle_gate.py` is **NOT** called by this new C++ node.
- RTK FLOAT 5 s pause/restore and all battery lockout checks in the
  final C++ safety gate; currently held in existing Python modules.
- Migration of Mission Manager, RPP, trajectory generator, spray,
  NTRIP, localization, and FastAPI/Socket.IO to C++.
- Offboard activation/exit lifecycle; this test package never arms,
  switches modes, starts missions, or launches itself.
- Full time-sync and topic-QoS confirmation on the actual hardware,
  dynamic obstacle gate, command replay testing, motor-direction
  and zero-on-boot tests.

**Physical motion is not authorized for this patch.** With the motors
disconnected, use read-only ROS echo and firmware uORB listener commands
to verify message decoding and timing. Keep `enable_motion=false` until
independent safety-gate implementation, safety review, and field checks.

## Build on Jetson or Ubuntu 22.04 / ROS 2 Humble

Your workspace is the root of the Jetson repository (it has `src/`).
First install the matching ROS 2 interfaces under `src/px4_msgs`:

```bash
cd ~/DYX_4WD_JETSON_DDS
git clone -b release/1.17 --depth 1 https://github.com/PX4/px4_msgs.git src/px4_msgs
# Confirm that custom-firmware message definitions match release/1.17.
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to dyx_px4_dds_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 pkg executables dyx_px4_dds_bridge
```

Do not `git add src/px4_msgs` inadvertently. Use `git -C src/px4_msgs rev-parse HEAD`
to pin the PX4 message ABI in deployment documentation, or use a separate
overlay workspace.

## Read-only bench check (not a motion test)

```bash
# With motors physically disconnected and rover disarmed:
ros2 run dyx_px4_dds_bridge dyx_px4_dds_bridge_node
# The node defaults to enable_motion=false.
# In another terminal:
ros2 topic echo /dyx_px4_dds_bridge/status
ros2 topic hz /fmu/in/trajectory_setpoint
ros2 topic echo /fmu/in/offboard_control_mode --once
ros2 topic echo /fmu/out/vehicle_status --once
```

Avoid running the new DDS publisher and the existing MAVROS command
bridge as active controllers at the same time.

## Applying and committing from VS Code

```bash
git status --short
git switch Jetson_DDS_Upgrade
git pull --ff-only origin Jetson_DDS_Upgrade
git apply --check /path/to/DYX_CPP17_DDS_PHASE1.patch
git apply /path/to/DYX_CPP17_DDS_PHASE1.patch
git diff --check
git add src/dyx_px4_dds_bridge docs/DYX_CPP17_DDS_PHASE1.md
git commit -m "feat(jetson): stage opt-in C++17 PX4 1.17 DDS bridge"
git push origin Jetson_DDS_Upgrade
```
