# DYX Autonomous — H6B native RPP preview integration
Prepared for engineering review | Baseline `ba697b3b80aa341e20dd1d49257aac952d7949d4`

## Implemented (non-actuating)
- New C++17 ROS2 `dyx_rpp_cpp17_native_preview` node (50 Hz wall timer) subscribes to PX4 `vehicle_local_position` and `vehicle_angular_velocity`, plus dynamic mission **preview-only** PointStamped goals.
- Preview goal topic `/dyx/mission/preview_goal_ned` expects `header.frame_id=px4_local_ned`, x=North, y=East; coordinates must be finite.
- Reuses existing `dyx_rpp_cpp17_core` heading conversion, telemetry validation and forward-yaw planner (the installed `dyx_rpp_cpp17_core` package must export its includes).
- Outputs **only** `std_msgs/String` diagnostics on `/dyx/rpp_cpp17_native_preview/status`; no `RppCommand`, Offboard, servo or PX4 input publishers.
- Default `preview_enabled=false`; enabled previews latch after stale telemetry, bad frame, PX4 estimator reset or time regression; restart required. No externally approved pivot certificate, so heading changes above pivot threshold remain WAIT_PIVOT.
- Adds ROS-free admission-gate C++ test plus C++17 static publication boundary test; no project Python test runner. Windows and Ubuntu offline compiles do not build the ROS2 node.

## Explicitly NOT delivered
- No actual mission backend or marking action, no authenticated goal ownership, no physical command, no full RPP feature-parity proof, no reverse-to-pivot implementation, no active control authority, no production launch/daemon change.
- No claim of functional production completion or PX4 v1.17 `px4_msgs` schema compatibility until ROS2 Humble colcon build is performed.
- Existing docs explicitly forbid reverse, unlike older reverse-pivot design requirements. This phase is intentionally non-actuating until that product decision is reconciled.

## Offline tests
Windows PowerShell: `powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/DYX_H6B_WINDOWS_OFFLINE_TEST.ps1`

Ubuntu WSL2: `bash scripts/DYX_H6B_UBUNTU_OFFLINE_TEST.sh`

ROS2 Humble build when ROS environment is available:
`source /opt/ros/humble/setup.bash && colcon build --packages-up-to dyx_rpp_cpp17_native_preview --symlink-install`
Build DOES NOT authorize launching a rover. Avoid running the preview node alongside an armed system before checking topic isolation.
