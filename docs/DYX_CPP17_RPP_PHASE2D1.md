# DYX 4WD — Phase 2D1: stationary position-window evidence (C++17)

Base: `dyxsatheesh/DYX_4WD_JETSON_DDS`, branch `Jetson_DDS_Upgrade`, after Phase 2C4 commit `5de4e4d`.

This patch ports **only** `PositionWindowStationaryDetector` from
`src/rpp_controller/rpp_controller/terminal_stop_regulator.py` to pure C++17.
It returns `PositionWindowEvidence` (stationary, held seconds, maximum observed
position displacement, maximum measured yaw rate, sample count).

The implementation faithfully retains the Python reference behavior:
- Default stationarity conditions in the larger Python controller: 0.50 s
  observation window, <=0.005 m max displacement and <=0.05 rad/s yaw rate,
  position sample gap <=0.20 s. These limits are passed explicitly to this class.
- Duplicate timestamps are ignored; backwards timestamps are rejected.
- Gaps, invalid measurements, movement and yaw-rate excursions clear evidence.
- The window preserves one sample just before the time cutoff and measures
  pairwise position displacements, not PX4 velocity estimates.
- It is **not** by itself a 20 mm marking-position certificate: this detector
  only establishes stationarity. Terminal identity, current radial error,
  settling timeouts and stop-failure latching remain in the Python
  `TerminalStopRegulator` and will be ported separately.

## Windows build/tests

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native -B build/dyx_rpp_native -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_rpp_native --config Release
ctest --test-dir build/dyx_rpp_native -C Release --output-on-failure
```

**Expected:** 13 CTests total (11 previous, 2 new). The new parity test imports
and replays the actual current Python RPP terminal-stop module from this
checkout. Run with CTest, not by executing the probe without arguments.

The new C++17 class does not publish ROS2 or DDS commands; it does not arm,
move, stop, reverse or spray. The old Python motion controller is unchanged.
A full PX4 rover marking accuracy claim requires real hardware testing and
independent ground-truth position checks.
