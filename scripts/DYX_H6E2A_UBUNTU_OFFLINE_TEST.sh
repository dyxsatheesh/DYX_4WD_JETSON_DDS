#!/usr/bin/env bash
# H6E2A: native C++17 read-only API adapter tests; no ROS2/network/actuation.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build="${HOME}/dyx_h6e2a_offline_build"
cmake -S "$repo/tools/dyx_h6e2_gate" -B "$build" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$build" -j2
ctest --test-dir "$build" --output-on-failure
printf '%s\n' 'H6E2A OFFLINE 3/3 TESTS PASSED; NO NETWORK, ROS2 OR ANDROID CERTIFICATION'
