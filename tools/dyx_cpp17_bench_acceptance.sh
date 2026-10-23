#!/usr/bin/env bash
# Batch136: single non-actuating Jetson+PX4 bench acceptance pass.
set -o pipefail
cd "$HOME/DYX_4WD_JETSON_DDS" || exit 2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export DYX_CPP17_AUTH_DB="$HOME/.config/dyx_native/bench_auth.sqlite3"
echo '=== GIT BRANCH / COMMIT ==='
git branch --show-current
git log -1 --oneline
echo '=== ALL C++17 NODE BUILDS ==='
colcon build --symlink-install --packages-select \
  mission_manager trajectory_generator rpp_controller dyx_px4_dds_bridge \
  spray_controller rtk_correction_bridge obstacle_monitor \
  --cmake-args -DBUILD_TESTING=ON || exit 3
echo '=== MISSION MANAGER REGRESSION SUITE ==='
ctest --test-dir build/mission_manager --output-on-failure || exit 4
echo '=== BENCH LAUNCH LOCKOUT CHECK ==='
python3 - <<'INNER'
from pathlib import Path
s=Path('src/rover_bringup/launch/dyx_cpp17_dds_bench.launch.xml').read_text()
for x in ('name="enable_motion" value="false"',
          'name="observe_only" value="true"',
          'name="serial_injection_enabled" value="false"',
          'name="enable_serial" value="false"'):
    if x not in s: raise SystemExit('FAIL: bench lockout missing: '+x)
print('PASS: motion, spray, injection and obstacle serial remain locked')
INNER
if [ "$?" -ne 0 ]; then exit 5; fi
echo '=== LIVE PX4 TOPIC DISCOVERY (NOT A VALIDITY CLAIM) ==='
for t in vehicle_status_v1 battery_status_v1 vehicle_local_position_v1 rover_rate_status vehicle_gps_position; do
  echo "--- $t ---"
  ros2 topic info "/fmu/out/$t" 2>&1 | head -4
done
echo 'PASS: non-actuating integration build/tests complete; sensor commissioning remains separate'
