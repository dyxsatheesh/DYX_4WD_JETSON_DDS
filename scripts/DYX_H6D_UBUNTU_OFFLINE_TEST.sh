#!/usr/bin/env bash
# WSL2/Linux C++17 offline tests, NEVER starts ROS, PX4 or hardware.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build="${HOME}/dyx_h6d_gate_build"
cmake -S "$repo/tools/dyx_h6d_gate" -B "$build" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$build" -j2
ctest --test-dir "$build" --output-on-failure
printf '%s\n' 'H6D OFFLINE TESTS PASSED; ROS2/PX4/HARDWARE NOT CERTIFIED'
