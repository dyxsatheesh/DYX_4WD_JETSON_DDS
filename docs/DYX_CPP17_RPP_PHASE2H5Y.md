# Phase 2H5Y — PX4 DDS read-only observation groundwork
Prepared for DYX Autonomous. New optional ROS 2 Humble/C++17 `dyx_jetson_readonly_dds` package subscribes to `/fmu/out/vehicle_status` and `/fmu/out/vehicle_local_position` using PX4 SensorDataQoS. It logs stale/fresh, armed and Offboard state once per second. **It never publishes on `/fmu/in/*`, does not issue commands, and is NOT auto-launched.**

In a future Jetson ROS 2 environment with version-matched `px4_msgs`, build with `colcon build --packages-select dyx_jetson_readonly_dds` and check topics while rover is disarmed and motor power disconnected. This C++17 source and its status guard are not a replacement for the existing opt-in `dyx_px4_dds_bridge`, GNSS fusion, battery/RTK safety logic or an independent hardware E-stop.

Windows/WSL pure-C++ tests validate observation freshness and permanent output disable. A static source test rejects known publisher/command APIs but cannot prove binary safety. ROS 2 ament build, PX4 ABI and actual DDS discovery remain untested without the required environment and receiver.
