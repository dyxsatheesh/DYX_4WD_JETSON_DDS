#!/usr/bin/env bash
# Phase 2H3D — read-only Jetson ROS 2/PX4 DDS monitor verification.
# Usage: bash scripts/verify_dds_monitor_2h3d.sh [ROVER_REPO] [WORKSPACE]
# This script never publishes commands, calls PX4 services, or touches AUX5/AUX6.
set -euo pipefail

ROVER_REPO="${1:-$HOME/DYX_4WD_JETSON_DDS}"
WORKSPACE="${2:-$HOME/dyx_dds_2h3d_ws}"
MONITOR_PACKAGE="$ROVER_REPO/src/dyx_dds_actuator_monitor"
PX4_MSGS="$WORKSPACE/src/px4_msgs"

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
note() { printf '\n== %s ==\n' "$*"; }

[[ -f /opt/ros/humble/setup.bash ]] || fail 'ROS 2 Humble not installed at /opt/ros/humble'
[[ -f "$MONITOR_PACKAGE/package.xml" ]] || fail "Monitor package missing: $MONITOR_PACKAGE"
[[ -f "$ROVER_REPO/src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/dds_actuator_monitor.hpp" ]] || fail 'Native monitor header missing'
[[ -f "$PX4_MSGS/msg/VehicleCommandAck.msg" ]] || fail "px4_msgs missing: clone PX4/px4_msgs release/1.17 into $PX4_MSGS"
for cmd in ros2 colcon git cmake timeout grep; do command -v "$cmd" >/dev/null 2>&1 || [[ "$cmd" == ros2 ]] || fail "Missing executable: $cmd"; done

note 'Source ROS 2 Humble and inspect the checked-out software'
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
command -v ros2 >/dev/null 2>&1 || fail 'ros2 CLI unavailable after sourcing Humble'
printf 'Rover HEAD: '; git -C "$ROVER_REPO" rev-parse --short HEAD || fail 'Rover repository unavailable'
printf 'px4_msgs branch: '; git -C "$PX4_MSGS" branch --show-current || fail 'px4_msgs repository unavailable'
[[ "$(git -C "$PX4_MSGS" branch --show-current)" == 'release/1.17' ]] || fail 'px4_msgs MUST be checked out on release/1.17 (no silent version mixing)'
printf 'px4_msgs HEAD: '; git -C "$PX4_MSGS" rev-parse --short HEAD

note 'Assert the v1.17 VehicleCommandAck message contains required fields'
for name in timestamp command result target_system target_component from_external; do
  grep -Eq "^[[:space:]]*(u?int(8|16|32|64)|bool)[[:space:]]+${name}([[:space:]]|$)" "$PX4_MSGS/msg/VehicleCommandAck.msg" || fail "Missing expected VehicleCommandAck field: $name"
  printf 'found %s\n' "$name"
done

note 'Build only the read-only monitor and its ROS 2 dependencies'
mkdir -p "$WORKSPACE"
cd "$WORKSPACE"
colcon build --base-paths "$PX4_MSGS" "$MONITOR_PACKAGE" \
  --packages-up-to dyx_dds_actuator_monitor \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"
ros2 pkg prefix dyx_dds_actuator_monitor >/dev/null || fail 'Monitor package not installed after build'
ros2 interface show px4_msgs/msg/VehicleCommandAck | sed -n '1,50p'

note 'Read-only ROS graph checks (PX4 agent may not yet be connected)'
ros2 topic list -t || true
ros2 topic info /fmu/out/vehicle_command_ack --verbose || true
ros2 service list -t | grep -F '/fmu/vehicle_command' || printf 'VehicleCommand service not discovered (not used by monitor)\n'

note 'Run monitor briefly; it has no publisher or service client'
set +e
timeout --signal=INT --kill-after=3s 12s \
  ros2 run dyx_dds_actuator_monitor dyx_dds_actuator_monitor_node
status=$?
set -e
if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then
  fail "Monitor process failed with status $status"
fi
printf '\nPHASE 2H3D: monitor compiled and launched. No actuator commands were sent.\n'
printf 'NOTE: ACK traffic/discovery depends on PX4, agent and network connectivity.\n'
