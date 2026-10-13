# DYX 4WD Rover — Phase 2H2A (Native C++17 Spray Transaction)

Prepared by: Satheesh | DYX Autonomous | 13 October 2026

## Objective
Port the safety-critical **mission SPRAY-only transaction contract** to pure C++17, independently testable in Windows Visual Studio or Linux GCC without Jetson, ROS 2, MAVROS or Python. This module **does not publish commands**, actuate motors, set PX4 outputs, or assert physical OFF.

## Defaults (pending hardware/PX4 calibration)
- AUX5 proposed PWM *intent* ON: **1400 µs**, OFF: **1200 µs**; servo press duration **3.0 s** measured from accepted PRESS ACK.
- 1.0-second PRESS ACK timeout; 5.0-second hard PRESS watchdog measured from command intent; 0.25-second recovery RELEASE retry.
- These are requested output-intent microseconds only. Do **not** assume a PX4 ActuatorSet normalized value corresponds to this PWM; validate PX4 output mapping, arming/disarming behavior, actuator allocation and physical servo endpoints on the hardware.
- Physical OFF must be independently confirmed before a new marking point can spray. A fresh, durable write-ahead journal lock for the exact mission identity must be recorded before transmitting PRESS.

## Workflow
1. Reject a mission PRESS unless the unique run-scoped goal identity, verified survey gate, stationary mission hold, PX4 armed/Offboard state, emergency-stop clearance, safety evidence and controller enable are all true. Default is BLOCKED.
2. Emit **one** PRESS intent for the point. Never retry PRESS if acknowledgment is lost or rejected.
3. On accepted PRESS ACK, start a **3-second** duration. Abort to RELEASE if safety/identity/mission hold is lost, if STOP is invoked, or if hard watchdog fires.
4. Emit RELEASE intent; only a matching positive RELEASE ACK produces a COMPLETED result. Release rejection/timeout retries RELEASE, latches a fault, and never reports success.
5. Mission adapter binds one release-confirmed result to the exact current terminal identity and delivers success/failed evidence to the native sequencer. No success without RELEASE ACK.
6. Restart with uncertain previous PRESS enters RELEASE-recovery and latches fault. Previously used identities stay blocked within the process. Production storage adapter must load all persisted used/uncertain identities.

## Limitations and next work
- This is a **non-actuating pure C++17 model**. No actual ROS2 DDS, PX4 commands, persistent journal IO, PWM calibration, laser mode, manual spray UI, or field validation is included.
- Crash-safe journal persistence, command transport acknowledgments, reboot replay denial, and explicit OFF feedback belong to the production adapter; a boolean supplied by an adapter must not be accepted as independent proof without validation.
- SOFTWARE ONLY cannot guarantee a fail-safe OFF if PX4 loses power or the servo is wired to retain its last command. Use independently validated hardware failsafe/enable circuitry, safe disarmed PWM configuration, and bench tests before field deployment.
- **There is no reverse-driving movement anywhere in this phase.**

## Tests
Four new native tests: happy cycle and one-shot marking, ACK timeout and E-stop, restart/release recovery and replay prevention, identity-matched mission completion. Combined expected native CTest count: 35.
