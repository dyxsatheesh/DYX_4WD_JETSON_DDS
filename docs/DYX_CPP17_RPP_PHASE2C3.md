# DYX 4WD — C++17 RPP Phase 2C3: tracking stability controller

**Branch:** `dyxsatheesh/DYX_4WD_JETSON_DDS` / `Jetson_DDS_Upgrade`.
**Base:** Phase 2C2 commit `33169c3628edcf48222ffd3c5e82fccb0f6058a6`.
**Status:** Standalone, non-actuating C++17 library; NOT field approved.

This ports `TrackingStabilityController` and its input/output/configuration
from `src/rpp_controller/rpp_controller/tracking_control.py` into
`include/dyx_rpp_cpp17_core/tracking_stability.hpp`. Its advisory output
includes acceleration authorization, recovery speed scaling, reset flags,
state transitions, and explicit invalid input reasons.

States are `tracking`, `recovery`, and `recapture_stable`. Recovery is entered
at the 50 mm cross-track / 15-degree heading thresholds (or bearing clamp)
by default. Stable tracking requires a 20 mm / 5-degree exit window and a
continuous 0.30 second recapture dwell. Sample time is capped at 0.10 second.
**These defaults mirror the current Python reference; they do not guarantee
20 mm physical path-tracking accuracy.** The C++ output cannot arm or actuate.

The existing `TrackingMetricsAccumulator`, diagnostics P95 histogram,
full ROS2 RPP adapter and offboard yaw-rate control are NOT migrated in this
patch. Python remains active and no launch configuration is changed.

## Windows VS Code parity tests

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native -B build/dyx_rpp_native -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_rpp_native --config Release
ctest --test-dir build/dyx_rpp_native -C Release --output-on-failure
```

The new `python_cpp_tracking_stability_parity` test imports the repository's
**actual** Python `tracking_control.py`, replays deterministic and seeded
randomized multi-cycle traces, and compares every public result field.
The C++ native smoke additionally verifies transition and input-fault rules.
The full suite should contain nine tests. Do not commit on failure.

## Next

Port the tracking metrics accumulator as a **separate** bounded-memory
component. Then proceed to the terminal radial-stop FSM, yaw-rate control,
ROS2 adaptation and sensor/safety checks before hardware actuation. Preserve
RTK FLOAT pause, battery/obstacle inhibit and RC takeover contracts.
