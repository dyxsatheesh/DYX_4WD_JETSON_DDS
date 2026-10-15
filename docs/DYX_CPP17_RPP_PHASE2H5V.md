# Phase 2H5V — Opt-in systemd process supervision
Prepared for DYX Autonomous. `tools/dyx_jetson_preflight/systemd/dyx-jetson-preflight.service` contains a hard-restricted observation-only service definition, deliberately without [Install]. Do not install or start it while rover power outputs are connected.

The C++17 executable sends `READY=1` and periodically sends `WATCHDOG=1` over systemd's local Unix datagram notification interface when configured. Startup refuses invalid watchdog settings or a broken notification endpoint. Handles SIGINT/SIGTERM gracefully; systemd can restart on failure.

No ROS, PX4, DDS publisher or actuator is created by this service. This watchdog monitors **process loop responsiveness only**; it does not prove that PX4 motors are stopped. OS-specific behavior and service file require Jetson validation later.
