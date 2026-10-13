# DYX 4WD / Jetson C++17 RPP — Phase 2C1 (precision guidance)

**Based on:** `Jetson_DDS_Upgrade` commit `594ed4e` (Phase 2B).

**Scope:** Port the complete `compute_precision_guidance` calculation from the
existing ROS-free Python module `rpp_controller/guidance.py` to C++17.
This is a mathematical library only: **no ROS node, no topic publishing, no
PX4 setpoints, no motor authority**. Do not enable the Phase 1 bridge for motion.

## What is identical to the Python reference

- Local ENU geometry (East x / North y), CCW-positive headings.
- Arc-length lookahead from monotonically tracked progress within the **active
  semantic marking span**.
- Steering-only virtual endpoint extension using the incoming span tangent;
  authoritative endpoint and remaining distance are never changed.
- Bearing fallback for a zero-length rover-to-target vector, moving command
  cone, left/right cross-track sign, heading error, and target-behind flag.
- Return diagnostics for later speed and yaw-rate algorithms.

## Windows VS Code test commands

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native -B build/dyx_rpp_native -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_rpp_native --config Release
ctest --test-dir build/dyx_rpp_native -C Release --output-on-failure
```

Five tests should appear: two Phase 2A/B Python parity tests, Phase 2B native
smoke, `python_cpp_precision_guidance_parity`, and `guidance_mock_smoke`.

The Phase 2C1 Python parity tests import the **live repository Python source**
and compare 21 fields, across >190 deterministic control cases including goal
overshoot and segment endpoints. A failed comparison blocks the commit.

## Next migration: Phase 2C2

Port `LongitudinalRegulator` including its stateful recovery hysteresis,
acceleration and deceleration caps, corner preview, curvature, and terminal
braking. Then port `TrackingStabilityController` and yaw-rate generation and
use ROS-bag/deterministic timeline replay to verify integration. Keep Python
as the reference until these are equivalent and hardware safety is proven.
