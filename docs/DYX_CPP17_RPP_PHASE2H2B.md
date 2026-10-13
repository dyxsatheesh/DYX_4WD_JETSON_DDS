# Phase 2H2B — Native C++17 Manual Spray Test

Prepared for DYX Autonomous 4WD rover. C++17, Windows-native testable, NON-ACTUATING.

## Purpose

Port the **manual spray test** policy from the existing Python spray controller to a
separate C++17 transaction that can never complete a mission marking point.
The PX4 transport, physical spray output, journal persistence, laser mode,
configuration UI and remote authorisation are not implemented by this phase.

## Safety contract

- AUX5 intent defaults to **ON = 1400 µs / OFF = 1200 µs**. The values are
  configuration *intent*, NOT an assertion about actual PWM output or about
  `MAV_CMD_DO_SET_ACTUATOR` normalized values.
- Manual ON requires authorized operator, healthy recent PX4 state, **PX4 armed**,
  released physical E-stop, controller enabled, mission idle, an exclusive
  AUX5 actuator lease and a previously
  verified OFF state. Manual testing does **not** require autonomous OFFBOARD.
- Every ON requires a unique request ID with a matching **durably recorded**
  request ID *before* the actuator adapter sends the PRESS command.
- Repeated ON requests do not reset the timer or send a second PRESS.
- The bounded hold is 3 seconds from an accepted PRESS acknowledgement, with
  an independent 5-second hard cutoff from the initial PRESS attempt.
- Manual OFF, STOP, E-stop, mission activation, loss of PX4 link/disarm or stale
  source immediately initiates RELEASE intent. Pending RELEASE is retried;
  timed out or rejected PRESS is never retried.
- A release acknowledgment is required for a manual success event. Such an ACK
  reports acceptance by transport/vehicle; **it is not physical servo feedback**.
  A subsequent manual PRESS requires fresh, separately verified OFF state.
- A crash with an uncertain PRESS restores into RELEASE-only recovery and a
  latched fault; the same request cannot be replayed within the running process.
- Manual successes and cancellations contain **no mission-completion verdict**.

## Test plan

Three new Windows-native CTest executables cover timed cycles, release ACK,
manual OFF, independent gates, stale PX4, journal mismatch, keepalive,
replayed IDs, PRESS timeout, delayed ACK, release retry, fault reset,
uncertain restart and watchdog.

## Production follow-up

1. Durable journal write/read/fsync and replay-safe request ID persistence.
2. Real PX4 DDS actuator request/ack adapter (with hardware-verifiable safe OFF),
   servo AUX5 mapping confirmation and no arm/disarm motion.
3. Laser mode: independently gated AUX6 logic without linking to spray state.
4. ROS 2 subscription/service integration, end-to-end safety, fault injection
   and physical validation on Jetson/Pixhawk 6X.

No Python controller, MAVROS, motor output or reverse-driving logic is added.
