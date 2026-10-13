# DYX 4WD — C++17 RPP Phase 2C4: tracking metrics

**Base:** `Jetson_DDS_Upgrade` at `62b3c12` (Phase 2C3).

**Status:** Non-actuating diagnostics library only. No firmware, active RPP,
ROS 2 launcher, or motor-command interface is modified.

## Faithful Python reference

`src/rpp_controller/rpp_controller/tracking_control.py`:
`TrackingMetricsAccumulator` and `TrackingMetricsSnapshot`.

Ported functions include: reset on mission/path identity change, finite-safe
sample rejection, mean/RMS speed and cross-track metrics, bounded trailing
P95, fixed-histogram whole-run P95 including overflow/saturation evidence,
monotonic projection regressions, recovery/recapture/cruise elapsed times,
explicit localization-discontinuity evidence and acceptance-invalid flag.

The C++ `TrackingMetricsConfig` contains the metrics-specific parameters
from Python's `TrackingControlConfig`; it uses the exact corresponding
Python defaults. This keeps diagnostics isolated from the tracking-control
state machine ported in Phase 2C3.

**Important:** `valid_for_acceptance` marks whether a run contains a
localization discontinuity; it does NOT certify 20 mm marking accuracy.

## Windows VS Code native test

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native -B build/dyx_rpp_native -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_rpp_native --config Release
ctest --test-dir build/dyx_rpp_native -C Release --output-on-failure
```

Two additional tests are provided:

- `python_cpp_tracking_metrics_parity`: deterministic stateful replay against
  the existing Python repository implementation, including histogram
  saturation, identity changes and discontinuities.
- `tracking_metrics_native_smoke`: bounded storage, counters, faults and reset.

All previous tests are preserved. Expected total with Phase 2C3: **11 tests**.
This patch has not been tested with the real rover or ROS2 DDS flight stack.

## Pending work

The precision terminal-stop state machine, yaw-rate policy, pivot/reverse
logic, controller ROS 2 C++ node and full Jetson telemetry/backend conversion
remain to be migrated and validated separately. Keep the existing Python RPP
active and keep the new C++ DDS bridge disabled during hardware safety review.
