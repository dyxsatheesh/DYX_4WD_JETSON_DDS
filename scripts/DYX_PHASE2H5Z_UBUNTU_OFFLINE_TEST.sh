#!/usr/bin/env bash
# Run in Ubuntu 22.04 / WSL2; does not touch actual Jetson or hardware.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$repo/tools/dyx_jetson_preflight" -B "$HOME/dyx_preflight_build" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$HOME/dyx_preflight_build" -j2
ctest --test-dir "$HOME/dyx_preflight_build" --output-on-failure
if [[ -d "$repo/src/dyx_rpp_cpp17_core/test/sqlite_lab" ]]; then
  cmake -S "$repo/src/dyx_rpp_cpp17_core/test/sqlite_lab" -B "$HOME/dyx_sqlite_lab_build" -DCMAKE_BUILD_TYPE=Debug
  cmake --build "$HOME/dyx_sqlite_lab_build" -j2
  ctest --test-dir "$HOME/dyx_sqlite_lab_build" --output-on-failure
fi
