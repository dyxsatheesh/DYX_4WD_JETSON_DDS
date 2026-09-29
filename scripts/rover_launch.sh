#!/usr/bin/env bash
# Start the rover stack: used by systemd/dyx-rover.service (and safe by hand).
#
# Exit 3 = another rover.launch.py is already running (the unit does not
# retry on 3). Exit 4 = the Pixhawk never appeared (the unit retries).
set -euo pipefail

WS=/home/flash/rover_ws
FCU_DEV="${DYX_FCU_DEV:-/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00}"
FCU_WAIT_SEC="${DYX_FCU_WAIT_SEC:-60}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

# Two stacks would both open the FCU port and both run a backend on :5001.
if pgrep -u "$(id -u)" -f "ros2 launch rover_bringup rover.launch.py" >/dev/null; then
    echo "rover.launch.py is already running (manual launch?); not starting a second one" >&2
    exit 3
fi

# MAVROS gives up if the FCU port is missing, so wait for USB enumeration.
for ((i = 0; i < FCU_WAIT_SEC; i++)); do
    [[ -e "$FCU_DEV" ]] && break
    sleep 1
done
if [[ ! -e "$FCU_DEV" ]]; then
    echo "Pixhawk not found at $FCU_DEV after ${FCU_WAIT_SEC}s" >&2
    exit 4
fi

cd "$WS"
exec ros2 launch rover_bringup rover.launch.py
