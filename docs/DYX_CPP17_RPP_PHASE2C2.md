# DYX 4WD — Phase 2C2: stateful longitudinal speed regulation

**Baseline:** `dyxsatheesh/DYX_4WD_JETSON_DDS` / `Jetson_DDS_Upgrade`, starting from commit `9aa1bdf`.

**Scope:** Pure, non-actuating C++17 port of the existing Python
`src/rpp_controller/rpp_controller/speed_regulator.py` module. The old Python
RPP node, all PX4 motor controls, ROS launch files and mission state remain
unchanged. The new module is not enabled or wired to DDS.

## Implemented

- Exact longitudinal min-of-caps precedence (including tie semantics).
- Mission/hardware ceilings, time and distance acceleration gates.
- Heading and cross-track recovery hysteresis, recovery entry/exit dwell.
- Corner braking preview, terminal braking preview and target-speed overrides.
- Tracking cap and acceleration inhibitor, curvature speed limit.
- Immediate hard-zero authority and per-leg reset semantics.
- Input/config validation and a structured result including cap ownership.

C++17 file: `src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core/speed_regulator.hpp`.

## Windows native build and parity test

From the root of your *existing* Jetson Git checkout in VS Code PowerShell:

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native -B build/dyx_rpp_native -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_rpp_native --config Release
ctest --test-dir build/dyx_rpp_native -C Release --output-on-failure
```

Expected **7 tests** (five existing tests plus
`python_cpp_speed_regulator_parity` and `speed_regulator_native_smoke`).
The Python test imports the *current repo's* real Python regulator; it runs
both Python and C++ through deterministic straight, corner, terminal, high
cross-track, recovery, tracking-inhibit, curvature and emergency-zero scenarios.
It compares 22 output values per cycle. Any difference is a failing test.

**Verification:** GCC C++17 compilation and native no-actuation smoke were
performed in the patch-preparation environment. Full Python-vs-C++ parity
requires running CTest in the complete Windows Git checkout; do not claim
Phase 2C2 validated or commit until all seven tests pass.

## Apply and push

```powershell
$Patch="$env:USERPROFILE\Downloads\DYX_CPP17_RPP_PHASE2C2.patch"
git apply --check $Patch
git apply $Patch
# Compile and run the seven tests above first.
git add src/dyx_rpp_cpp17_core docs/DYX_CPP17_RPP_PHASE2C2.md
git diff --cached --check
git commit -m "feat(jetson): port RPP speed regulator to C++17"
git push origin HEAD:refs/heads/Jetson_DDS_Upgrade
```

## Production-readiness hold

This Phase 2C2 port does not demonstrate precise stop distance or 800 mm
acceleration/deceleration performance; those are system-level requirements
requiring controller integration, measured vehicle braking calibration,
mission replay and field tests. The terminal radial stop, yaw-rate steering,
reverse-pivot, RTK/battery gates and full ROS2 node remain unported.
