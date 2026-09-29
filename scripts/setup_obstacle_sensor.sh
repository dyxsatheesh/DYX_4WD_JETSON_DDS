#!/bin/bash
# One-time setup for the forward ultrasonic obstacle sensor (DYP A21, UART).
#
#   ./scripts/setup_obstacle_sensor.sh            # build, find the sensor, save its port
#   ./scripts/setup_obstacle_sensor.sh --enable   # ...and switch detection ON
#
# Run as the rover user (flash), no sudo. Safe while the stack is running,
# but restart the stack afterwards so the new node starts.
#
# Wiring (same sensor as the 3WD NRP_ROS rover): sensor VCC -> 5 V, GND ->
# GND, sensor TX -> Jetson UART RX. On the Orin Nano 40-pin header that is
# pin 10 (RX) = /dev/ttyTHS1 (pin 8 = TX, not needed). Check the A21 TX logic
# level against the Jetson's 3.3 V input.
#
# Behaviour once enabled: danger (default <= 1.5 m) pauses a RUNNING mission,
# manual resume only; warning (<= 3.0 m) is announced; a dead sensor is a
# warning only. Zones: POST /api/obstacle/config or the settings file below.

set -uo pipefail
ENABLE=false
[[ "${1:-}" == "--enable" ]] && ENABLE=true
SETTINGS="${DYX_OBSTACLE_SETTINGS_PATH:-$HOME/.ros/dyx_obstacle_settings.json}"
WS="${ROVER_WS:-$HOME/rover_ws}"

if [[ $EUID -eq 0 ]]; then
    echo "Run as the rover user, not root." >&2
    exit 1
fi

echo "== 1/4 Build obstacle_monitor (new package) and rover_bringup"
# ROS setup scripts read unset variables; relax nounset only while sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
( cd "$WS" && colcon build --symlink-install --packages-select obstacle_monitor rover_bringup ) || {
    echo "Build FAILED" >&2
    exit 1
}

echo
echo "== 2/4 Access"
id -nG | grep -qw dialout && echo "  $USER is in dialout" \
    || echo "  WARNING: $USER is not in dialout; run: sudo usermod -aG dialout $USER (then log in again)"
python3 -c "import serial" 2>/dev/null && echo "  pyserial OK" \
    || echo "  WARNING: pyserial missing; run: sudo apt-get install -y python3-serial"

echo
echo "== 3/4 Find the sensor (3 s per port, both A21 baud rates)"
FOUND_PORT=""
FOUND_BAUD=""
for port in /dev/ttyTHS1 /dev/ttyTHS2 /dev/ttyTHS0 /dev/ttyUSB*; do
    [[ -e "$port" ]] || continue
    if users=$(fuser "$port" 2>/dev/null) && [[ -n "$users" ]]; then
        owner=$(ps -o cmd= -p ${users%% *} 2>/dev/null | cut -c1-60)
        echo "  $port in use ($owner) - skipped"
        continue
    fi
    for baud in 115200 9600; do
        result=$(python3 - "$port" "$baud" <<'PY'
import sys, time, serial
port, baud = sys.argv[1], int(sys.argv[2])
try:
    s = serial.Serial(port, baud, timeout=0.2)
except Exception as e:
    print(f"0 cannot-open"); sys.exit()
buf = bytearray(); frames = []; t0 = time.time()
while time.time() - t0 < 3:
    buf += s.read(64)
    while len(buf) >= 4:
        if buf[0] == 0xFF and ((0xFF + buf[1] + buf[2]) & 0xFF) == buf[3]:
            frames.append((buf[1] << 8) | buf[2]); buf = buf[4:]
        else:
            buf = buf[1:]
s.close()
print(len(frames), frames[:5])
PY
)
        count="${result%% *}"
        echo "  $port @ $baud: $result"
        if [[ "$count" =~ ^[0-9]+$ && "$count" -ge 5 && -z "$FOUND_PORT" ]]; then
            FOUND_PORT="$port"; FOUND_BAUD="$baud"
        fi
    done
done

echo
echo "== 4/4 Save settings -> $SETTINGS"
if [[ -z "$FOUND_PORT" ]]; then
    echo "  No sensor frames found. Check power and wiring (sensor TX -> Jetson RX)."
    echo "  (If obstacle_monitor already holds the port, its readings are in"
    echo "   GET /api/obstacle/status.)"
    exit 1
fi
python3 - "$SETTINGS" "$FOUND_PORT" "$FOUND_BAUD" "$ENABLE" <<'PY'
import json, os, sys
path, port, baud, enable = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4] == "true"
os.makedirs(os.path.dirname(path), exist_ok=True)
try:
    settings = json.load(open(path))
except Exception:
    settings = {}
settings.update(port=port, baud=baud)
settings.setdefault("danger_mm", 1500)
settings.setdefault("warning_mm", 3000)
settings["enabled"] = True if enable else bool(settings.get("enabled", False))
tmp = path + ".tmp"
with open(tmp, "w") as handle:
    json.dump(settings, handle, sort_keys=True)
os.replace(tmp, path)
print(f"  saved: {settings}")
PY
echo
echo "Done. Restart the rover stack, then check GET /api/obstacle/status."
[[ "$ENABLE" == true ]] || echo "Detection is OFF until enabled (tablet, or re-run with --enable)."
