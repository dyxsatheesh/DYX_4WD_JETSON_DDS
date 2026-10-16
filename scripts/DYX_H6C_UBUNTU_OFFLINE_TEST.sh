#!/usr/bin/env bash
# Pure C++17 / no application Python; this does NOT run ROS or actuation.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build="${HOME}/dyx_h6c_gate_build"
cmake -S "$repo/tools/dyx_h6c_gate" -B "$build" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$build" -j2
ctest --test-dir "$build" --output-on-failure
printf '%s\n' 'H6C OFFLINE VALIDATED; ROS2 COLCON AND PX4 NOT CERTIFIED'
