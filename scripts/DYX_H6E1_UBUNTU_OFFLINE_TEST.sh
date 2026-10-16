#!/usr/bin/env bash
# ROS-free/offline native C++17 contract inventory; never starts runtime.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build="${HOME}/dyx_h6e1_api_audit_build"
cmake -S "$repo/tools/dyx_h6e_api_parity" -B "$build" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$build" -j2
ctest --test-dir "$build" --output-on-failure
"$build/dyx_h6e_api_audit" --root "$repo" --json "$build/dyx_h6e1_api_parity.json"
printf 'Report: %s\n' "$build/dyx_h6e1_api_parity.json"
printf '%s\n' 'NOT CERTIFIED: source route counts do not demonstrate Android protocol parity.'
