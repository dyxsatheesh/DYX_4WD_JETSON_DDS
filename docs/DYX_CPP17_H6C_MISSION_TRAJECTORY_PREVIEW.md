# DYX Autonomous — H6C mission/trajectory preview integration
Engineering note | baseline `40f4eae48ceeab2924b879b5ffd1e9566072c4a5`

## What is implemented
- A new optional ROS 2 Humble C++17 application, `dyx_mission_cpp17_preview`, reads a fixed NED list from parameters (north,east flat pairs, **not latitude/longitude**), compiles it with the existing `NativeTrajectoryBuilder` (extension OFF by default), and binds marking anchors to an existing `MissionSequencer` in **READY** state only.
- Selects a single operator-specified marking point and emits `geometry_msgs/PointStamped` every 500 ms on `/dyx/mission/preview_goal_ned` with frame `px4_local_ned`; H6B consumes this topic for its diagnostic preview if separately enabled. This component is not auto-launched.
- Checks limits, finite coordinates, count/selection, unique consecutive position, raw index mapping and exact marking coordinates; includes Windows and Ubuntu ROS-free C++17 tests and static forbidden-publisher mutation checks.
- Default `preview_enabled=false`. Disabled component emits *only* a diagnostic status. The path, index and extension parameters are read only at startup; no auto next/skip/resume, mission start, spray transaction or motor output exists.

## Important operational limits
- Input is LOCAL PX4 NED positions; the caller is responsible for transforming GNSS/marking reference into PX4 local coordinates. No coordinate transform or GNSS lever arm is supplied by H6C. The goal z=0 is preview-only, not a validated height.
- The sequential mission READY state is a **diagnostic binding**. It does not certify raw RTK FIX, marking-point accuracy, point arrival, physical stationary state or spraying.
- `extension_enabled=true` compiles optional dummy alignment geometry using existing `NativeTrajectoryBuilder`, but the preview publishes only the original marking coordinate, never a dummy point. Do not interpret as complete trajectory tracking or extension operation.
- The compiled ROS 2 component has NOT been colcon built or run against PX4. Offline tests build only the selector and static publication gate; they do not certify full runtime integration or feature parity.
- Existing production `rover.launch.py` and `dyx-rover.service` are intentionally untouched. This is not H6C production cutover and not hardware acceptance.

## Test commands
Windows PowerShell:
`powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\DYX_H6C_WINDOWS_OFFLINE_TEST.ps1`

Ubuntu WSL2:
`bash scripts/DYX_H6C_UBUNTU_OFFLINE_TEST.sh`

Optional ROS 2 Humble compile (not an authorization to run hardware):
`source /opt/ros/humble/setup.bash && colcon build --packages-up-to dyx_mission_cpp17_preview --symlink-install`

Until a full safety-arbitrated command owner and feature parity are built, keep the physical rover disconnected or inhibited. Do not run legacy MAVROS and DDS control together.
