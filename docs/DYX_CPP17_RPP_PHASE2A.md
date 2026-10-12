# DYX 4WD — C++17 RPP Phase 2A: math parity

**Target repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS` / `Jetson_DDS_Upgrade`

**Status:** Non-actuating library, not a production RPP controller. No launch or PX4 changes.

This patch ports seven small *pure* functions faithfully from the current Python source:

| C++17 primitive | Python source |
| --- | --- |
| `wrap_angle`, `project_onto_segment` | `src/rpp_controller/rpp_controller/path_geometry.py` |
| `adaptive_lookahead_m`, `wrap_heading_error`, `limit_bearing_to_moving_cone` | `src/rpp_controller/rpp_controller/guidance.py` |
| `braking_distance`, `allowable_speed_for_distance` | `src/rpp_controller/rpp_controller/speed_regulator.py` |

These use the same local ENU geometry as Python (`x = East`, `y = North`),
positive cross-track to the LEFT of travel, and positive yaw CCW. They do not
write DDS commands; the v1.17 bridge remains separate and opt-in.

## Native Windows / VS Code parity test (no ROS needed)

From the repository root, with CMake, Python and C++17 toolchain installed:

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native -B build/dyx_rpp_native
cmake --build build/dyx_rpp_native --config Release
ctest --test-dir build/dyx_rpp_native -C Release --output-on-failure
```

CTest calls `python_parity.py`, imports the real current Python RPP source
from this repository and compares it with the C++ executable. An error is a
FAIL, not permission to change the existing Python calculation.

## Jetson ROS 2 Humble build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dyx_rpp_cpp17_core
```

## Follow-up tasks, NOT accomplished by this patch

Port `PathGeometryIndex` semantic span, progress tracking and monotonic
projection, then complete `compute_precision_guidance` including endpoint
extension. Port `LongitudinalRegulator` state machine, `TrackingStabilityController`,
terminal radial-stop certification, reverse-pivot FSM and full RPP ROS2 callbacks.
Replay known mission/rosbag test fixtures and verify equivalent outputs across
straight legs, overshoots, RTK FLOAT, faults and repeated pivots before adding
the C++ controller to the rover launch.

Never remove Python RPP, disable emergency stop, or arm during a math-only test.
