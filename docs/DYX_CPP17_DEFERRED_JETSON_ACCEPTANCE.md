# DYX 4WD Rover — Deferred Jetson / hardware verification register

**Prepared by Satheesh | 13 October 2026**

The Jetson Orin Nano and rover hardware are not available for testing during
this development period. Continue ROS-free C++17 development using the native
Windows Visual Studio CTest lane. Do not confuse offline green tests with
commercial deployment validation.

| Item | Offline status | When hardware is available |
|---|---|---|
| Phase 2H3D ROS 2 package | Windows 51/51 tests passed; script committed `4253fcb` | Jetson Ubuntu 22.04 ROS 2 Humble `colcon build`, `px4_msgs release/1.17` compatibility |
| PX4 XRCE-DDS link | Native command encoding/monitor tested | Agent over Holybro baseboard internal Ethernet, topic QoS, link loss, reboot, service discovery |
| DDS read-only monitor | C++17 monitoring logic validated offline | `ros2 run dyx_dds_actuator_monitor dyx_dds_actuator_monitor_node`, read-only ACK observation |
| Actuator command mapping | OFF/ON intents only | With outputs safe, verify PX4 AUX5/AUX6 setup, normalized values, physical OFF on arm/disarm, stop and power loss |
| Spray journaling/recovery | Offline state-machine tests | Simulated crashes, durable fsync and reboot, release recovery, real physical OFF sensor or independent proof |
| Rover motion/offboard | Offline guidance/mission state-machine tests | Verify zero commands, PX4 failsafes, RC takeover, speed/yaw-rate authority; no reverse motion |
| RTK precision | Offline raw GNSS gate logic | Rear master antenna at each Pn; post-stop fresh RTK FIXED <=30 mm; actual surveyed marking accuracy |
| Battery | Native 22 V debounce tested | 8S LiFePO4 discharge tests, 30 s debounce, persistent lockout, RC-only recovery, no unsafe restart |
| GNSS correction/source health | Native input freshness logic tested | Verify real NTRIP injection, mosaic/UM982 configuration as installed, selected GNSS device and timestamp behavior |
| Obstacle / emergency stop | Native blocking logic tested | Independent STOP path, sensor failure, stale telemetry, actuator isolation, external e-stop hardware |
| Android integration | Not validated by native tests | Production Kotlin/Compose or selected client UI; START/PAUSE/RESUME/NEXT/SKIP/STOP permissions |

The Windows build must never publish or energize rover outputs. Jetson tests
must begin **monitor-only**, with AUX5/AUX6 disconnected/disabled, and proceed
to any physical activation only under a separate signed hardware-safety plan.
The final production system must be assessed as a whole, not by individual
module smoke-test counts.
