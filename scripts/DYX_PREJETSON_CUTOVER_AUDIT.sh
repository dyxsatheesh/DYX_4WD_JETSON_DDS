#!/usr/bin/env bash
# Offline development-only audit; never starts ROS, PX4, motors or sprayers.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build="${HOME}/dyx_cutover_audit_build"
cmake -S "$repo/tools/dyx_prejetson_cutover" -B "$build" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$build" -j2
ctest --test-dir "$build" --output-on-failure
"$build/dyx_cutover_audit" --root "$repo" --json "$build/dyx_cutover_status.json"
echo "Report: $build/dyx_cutover_status.json"
echo "WARNING: this reports missing feature parity; it does not certify a release"
