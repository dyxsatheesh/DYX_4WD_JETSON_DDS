# DYX Autonomous — Phase H6A: isolated native DDS observation bringup
Prepared by Satheesh | 15 October 2026 | apply only to `c862998` on `JETSON_DDS_Upgrade`

## Scope: not a production cutover
This package stages a ROS 2 XML launch with **two** existing C++17 observers. It adds no motor/spray/mission commands, no PX4 Offboard publisher and no automatic service enablement. The production `rover.launch.py` and `dyx-rover.service` stay untouched. It does NOT retire any of the 11 legacy cutover blockers, because it does not yet replace mission, NTRIP, backend or full RPP execution.

1. `dyx_jetson_readonly_dds_node` subscribes to PX4 telemetry; it does not publish ROS command topics.
2. `dyx_rpp_cpp17_shadow_node` starts with `shadow_enabled=false`; it publishes local diagnostics only, not motor commands. Optional preview calculations in the existing shadow remain non-actuating.
3. A ROS-free C++17 static safety gate and mutation tests validate that the staged XML includes only the two allowed nodes and the checked-in sources remain non-actuating. This is a source check, **not** run-time proof.

## Offline validation before Jetson
- Windows PowerShell: `powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/DYX_H6A_WINDOWS_OFFLINE_TEST.ps1`
- Ubuntu/WSL2: `bash scripts/DYX_H6A_UBUNTU_OFFLINE_TEST.sh`
- Keep the prior pre-Jetson audit and H5Z suites. They still report legacy production dependencies.

## ROS2 Humble integration validation (Ubuntu ROS2 environment, if installed)
Source `/opt/ros/humble/setup.bash`, build the workspace with `colcon build --symlink-install --packages-up-to dyx_native_observe_bringup`, then source `install/setup.bash`. The `px4_msgs` package MUST match your firmware's exported `dds_topics.yaml` and PX4 v1.17 message definition. Check every dependency; this static phase does not guarantee the workspace builds with the target message versions.

For a controlled ROS2 observation test only, with **no motors or sprayer power** and only after checking no old rover launcher is active:

`ros2 launch dyx_native_observe_bringup observe.launch.xml`

Expect read-only logs. No `/fmu/in` publisher is provided by this launch. PX4 telemetry absence may be normal in a simulated/offline environment; it does not prove Ethernet DDS has connected.

## Remaining work after H6A
Actual C++17 production RPP and mission ROS2 executors, trajectory and GNSS/NTRIP, backend/API, battery and obstacle integration, spray/laser authority, XML production launch and reliable systemd, and feature-by-feature parity/hardware tests. This phase does not count them as done.

## Safety and change management
Do not enable `dyx_px4_dds_bridge` motion, MAVROS or other actuator publishers when testing this observation-only launch. This package must never be installed as the production boot service as a substitute for complete system safety. Commit only after both offline suites pass. Do not reapply this patch on an already modified branch.
