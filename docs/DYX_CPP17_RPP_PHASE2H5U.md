# Phase 2H5U — Real Linux C++17 observation-only executable
Prepared for DYX Autonomous. No Jetson necessary for compilation (WSL2/Linux).

Independent executable under `tools/dyx_jetson_preflight`. Compiles without ROS, MAVROS, PX4 or motor libraries. `--check` reports **software assertions**, not physical inhibition; `--run` stays in an observer process until SIGTERM/SIGINT. No network listener, DDS writer or actuator output is constructed.

Build: `cmake -S tools/dyx_jetson_preflight -B build/dyx_preflight -DCMAKE_BUILD_TYPE=Debug && cmake --build build/dyx_preflight && ctest --test-dir build/dyx_preflight --output-on-failure`

Do not deploy on an attached or armed rover before separate hardware safety review. Later phases will add opt-in diagnostics and service supervision, never motion authority.
