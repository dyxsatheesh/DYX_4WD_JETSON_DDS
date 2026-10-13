# Phase 2H3D — Jetson ROS 2 C++17 DDS monitor build and compatibility verification

**DYX Autonomous | Prepared by Satheesh | 13 October 2026**

**Baseline:** `Jetson_DDS_Upgrade`, Phase 2H3C commit `ccf9331`.

## Scope

Phase 2H3D adds a **read-only Jetson build/diagnostics script**, not a live actuator control feature. The established Windows 51-test native lane is unchanged. The ROS 2 C++ node must additionally compile against the real ROS 2 Humble and **PX4 1.17-compatible** `px4_msgs` definitions. No ROS 2 compilation was possible in the offline Windows lane.

The PX4 v1.17.0 and customized DYX `dds_topics.yaml` both publish `/fmu/out/vehicle_command_ack` as `px4_msgs::msg::VehicleCommandAck`. The `PX4/px4_msgs` `release/1.17` message defines `timestamp`, `command`, `result`, `target_system`, `target_component` and `from_external`, which the monitor reads. ROS graph discovery for `/fmu/vehicle_command` is **diagnostic only**, not required to operate the read-only monitor.

## Prerequisites on Jetson Orin Nano

- Ubuntu 22.04 / JetPack 6.2.2, ROS 2 Humble with development tools (`colcon`, `ament_cmake`, `rclcpp`, `rosidl`).
- Current rover checkout on `Jetson_DDS_Upgrade`.
- Matching PX4 message repository branch `release/1.17`.
- PX4 XRCE-DDS Agent configured and connected if live topics/services should appear.
- **Keep spray/laser power disconnected or outputs made physically safe** during initial verification. No motor or tool actuation is required.

## Explicit, limited workspace setup (no Python app launch, no MAVROS)

```bash
source /opt/ros/humble/setup.bash
cd ~/DYX_4WD_JETSON_DDS
git status --short
git pull --ff-only origin Jetson_DDS_Upgrade
mkdir -p ~/dyx_dds_2h3d_ws/src
# Use the matching PX4 1.17 message definitions, NOT px4_msgs/main.
git clone --branch release/1.17 --single-branch \
  https://github.com/PX4/px4_msgs.git ~/dyx_dds_2h3d_ws/src/px4_msgs
bash scripts/verify_dds_monitor_2h3d.sh \
  ~/DYX_4WD_JETSON_DDS ~/dyx_dds_2h3d_ws
```

If `~/dyx_dds_2h3d_ws/src/px4_msgs` already exists, inspect its branch and commit rather than cloning on top of it. The script fails closed if the wrong branch is present.

The script checks the exact v1.17 message fields, compiles and installs only `px4_msgs` plus `dyx_dds_actuator_monitor`, inspects ROS topics/services, and runs the monitor for a bounded period. It publishes nothing and cannot send commands to PX4.

## Acceptance

1. Jetson C++17 `colcon build` succeeds for `dyx_dds_actuator_monitor`.
2. `ros2 interface show px4_msgs/msg/VehicleCommandAck` includes all fields read by the monitor.
3. Monitor starts and prints `DRY RUN ... NO publish/call path`.
4. With PX4/Agent connected, `ros2 topic info /fmu/out/vehicle_command_ack --verbose` shows the topic and matching subscriber. A command-187 ACK is **not** required; none should be deliberately triggered.
5. DDS `VehicleCommand` service availability, if any, is observational only.
6. The 51 existing Windows C++17 tests remain unchanged and must still pass after patch application.

## Safety limitations

A command ACK is not an actuator position sensor and has no unique transaction identity. The monitor does not call `VehicleCommand` services or publish command/setpoint topics. Read-only DDS topic success does not prove GNSS, RTK, servo off-state, independent emergency stop, or commercial deployment readiness. For physical actuation, a separately reviewed command/ACK/physical feedback path is required.
