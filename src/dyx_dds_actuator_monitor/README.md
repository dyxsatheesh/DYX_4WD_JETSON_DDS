# DYX DDS actuator monitor (Phase 2H3C)

ROS 2 Humble C++17 read-only monitor. Observes `/fmu/out/vehicle_command_ack`
using BEST_EFFORT/VOLATILE QoS and performs ROS graph discovery for the
`/fmu/vehicle_command` service. No commands, setpoints, or service requests
are published/sent. A raw command=187 `VehicleCommandAck` cannot correlate
individual spray or laser transactions; no positive receipt is fabricated.

ROS 2 build, **after** matching `px4_msgs` version to the actual PX4 firmware:

```bash
source /opt/ros/humble/setup.bash
cd ~/DYX_4WD_JETSON_DDS
colcon build --packages-select dyx_dds_actuator_monitor --symlink-install
source install/setup.bash
ros2 run dyx_dds_actuator_monitor dyx_dds_actuator_monitor_node
```

The Windows `native_cpp_only` CMake/CTest lane tests the ROS-free monitor
safety logic but does not compile this ROS 2 node. A passing Windows test
is not an end-to-end XRCE-DDS or Pixhawk 6X test.

Production restrictions: Keep AUX5/AUX6 disconnected while testing. This
package may never be repurposed into a command emitter without a separate
review and real command/ACK correlation via PX4's request/reply service.
