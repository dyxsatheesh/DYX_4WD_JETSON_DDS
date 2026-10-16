# DYX Autonomous - Functional C++17 node organization

Target branch: Jetson_DDS_Upgrade_Organized. Based on 739f933.

| Python original | C++17 native | Status |
|---|---|---|
| trajectory_generator | dyx_trajectory_generator | Native builder diagnostic only |
| mission_manager | dyx_mission_manager | Native READY-state preview only |
| rpp_controller | dyx_rpp_controller | Native 50 Hz preview only |
| cmd_vel_bridge | dyx_px4_dds_bridge | Existing gated/disabled DDS bridge |
| spray_controller | dyx_spray_controller | Native spray transaction observer only |
| rover_backend | dyx_backend_cpp17_legacy_readonly | Native H6E2A library; runtime missing |
| rtk_correction_bridge | pending | Python fallback |
| obstacle_monitor | pending | Python fallback |
| localization | pending | Python fallback |
| rover_bringup | pending | Python production launch unchanged |

Existing dyx_rpp_cpp17_core, older preview packages and all Python source are intentionally retained.
They provide tested algorithms and regression tests. This does not finish full package ownership refactoring.
Do NOT launch old and new publishers together, or enable motion/spray from these previews.

On Jetson / Ubuntu with ROS2 Humble:

    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-up-to dyx_trajectory_generator dyx_mission_manager dyx_rpp_controller dyx_spray_controller dyx_px4_dds_bridge

No colcon or hardware tests have been executed by this organization patch.