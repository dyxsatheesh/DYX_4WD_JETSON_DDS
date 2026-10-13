# DYX Autonomous | Phase 2H2C — Native C++17 AUX6 laser marking

Prepared for the DYX 4WD marking rover. Offline Windows-testable, **NON-ACTUATING** C++17. No user-authored Python, MAVROS or reverse-driving logic is introduced.

## Scope and hardware contract

- Laser uses the **separate AUX6 / actuator set 2** intent. AUX5 remains reserved for spray; the laser state machine does not access or command AUX5.
- The existing reference controller proposes normalized **ON = +1.0**, **OFF = -1.0**, and `PWM_AUX_FUNC6 = 302` for PX4. Values here are transport-neutral *intent* only, **not proven physical PWM, laser power, actuator mapping or shutdown**. Confirm PX4 output assignment, disarmed output and physical failsafe before activation.
- Steady mode: a precisely stopped and raw RTK-FIXED-validated marking point may request ON. After accepted ON command, controller emits `LaserEvent::on_acknowledged` (transport acknowledgment only). Mission manager must independently decide whether a marking is complete; `LaserOutput::marking_completed` is always false.
- Steady ON has **no time limit while exact run/point hold remains valid**. Before hold topic handoff, there is a **1.5-second** bound while the same current mission goal remains live. When NEXT/SKIP/STOP clears hold, send OFF.
- Blink mode: raw GNSS indicates **outside tolerance** and the *exact same* run/point has an explicit BLINK hold. Command ON and OFF alternately, each for **0.5 s following acknowledgment**. Blink **never** reports a marking complete. This is a status indication, not a spray/mark.
- No overlapping ON/OFF requests while an acknowledgment is pending. A stale/mismatched ACK cannot complete a command. ON rejection or timeout initiates OFF-only recovery, never retries ON. OFF rejection or timeout initiates bounded-interval OFF retry and latches a fault.
- STOP, E-stop, loss of RTK/PX4/offboard, invalid timestamps, controller disable, point-identity mismatch or lost hold result in OFF intent or conservative OFF recovery. Actual safe physical shutdown requires a hardware-independent kill / disarmed-off configuration.
- Before the first ON, a **durable write-ahead journal**, exclusive **AUX6 lease**, and **independent OFF verification** are mandatory. A restart with uncertain ON state enters OFF-only fault recovery and blocks replay of that identity. A fresh independently verified OFF state is required before another goal is allowed to start. Note that PX4 command acknowledgment alone is not physical OFF feedback.

## Code

- `include/dyx_rpp_cpp17_core/laser_marking.hpp`: stand-alone typed laser mode/state/event and native FSM.
- `test/laser_steady_native_smoke.cpp`: ON ACK, bounded handoff, unlimited validated hold, OFF on NEXT, no duplicate.
- `test/laser_blink_native_smoke.cpp`: blink timing, serialized acknowledgments and identity-bound hold release.
- `test/laser_safety_native_smoke.cpp`: fail-closed gating, late ACK, STOP, E-stop, OFF retry, uncertain restart and fault reset.

## Windows verification

```
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected after this patch: **41/41** tests. The prior 38 tests remain. All tests are ROS/Python-free and do not actuate hardware.

## Remaining production integration

1. Implement authenticated ROS2 DDS transport and PX4 AUX6 actuator mapping, plus hardware-verified disarmed OFF and emergency power-cut behavior.
2. Enforce the AUX5/AUX6 exclusive hardware leases, persistent write-ahead logging and point replay prevention across restarts.
3. Supply fresh trusted raw RTK survey, stationary/mission hold, PX4 link and safety data; correlate live acknowledgment sequence IDs in the same monotonic clock.
4. Verify STOP, E-stop, unexpected restart and GNSS/PX4 outages on a bench **without energizing the laser** before live field tests.

This patch is not a production-ready physical laser controller and confers no ability to energize hardware.
