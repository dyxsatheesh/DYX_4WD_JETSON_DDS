#!/usr/bin/env bash
# Offline C++17 test only. Never launches ROS2, publishes DDS, arms motors or spray.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build="${HOME}/dyx_h6a_offline_build"
cmake -S "$repo/tools/dyx_h6a_gate" -B "$build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build" -j2
ctest --test-dir "$build" --output-on-failure
"$build/dyx_h6a_gate" "$repo"
echo "H6A STATIC ONLY; ROS2 colcon and target PX4 compatibility not yet tested."
