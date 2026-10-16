#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build="${HOME}/dyx_h6b_gate_build"
cmake -S "$repo/tools/dyx_h6b_gate" -B "$build" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$build" -j2
ctest --test-dir "$build" --output-on-failure
echo 'H6B OFFLINE PREVIEW GATE PASSED; ROS 2 COLCON/HARDWARE NOT CERTIFIED'
