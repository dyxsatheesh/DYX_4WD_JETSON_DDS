# Phase 2H5Z — Pre-Jetson offline bench / failure injection pack
Prepared for DYX Autonomous, 15 October 2026.

This phase adds REAL process-level Linux AF_UNIX HTTP tests: HMAC success/failure, SQLite replay across two process incarnations, second-owner denial, invalid key permissions, and observation-only status assertions. The other local tests cover the executable, systemd notification, cryptographic parsing, SQLite replay and PX4 read-only observation contract.

**Ubuntu pre-hardware verification**: `bash scripts/DYX_PHASE2H5Z_UBUNTU_OFFLINE_TEST.sh` (g++, cmake, SQLite3 dev, OpenSSL dev, Python3 required). It runs legacy SQLite tests plus new preflight tests.

**Windows pre-hardware verification**: `powershell -ExecutionPolicy Bypass -File scripts/DYX_PHASE2H5Z_WINDOWS_TEST.ps1`; legacy Windows C++ suite is unchanged and new cross-platform smoke tests run separately.

**Hardware acceptance still PENDING**: ROS 2 ament build with firmware-matched PX4 message types; internal Ethernet XRCE-DDS discovery; every 4WD rover mission feature's C++17 parity; RTK fixed/float behavior; RoboClaw/encoder control; 22.0V battery lockout; spray PWM and arm/disarm; Android API compatibility; site accuracy targets; real watchdog/physical STOP; systemd install, crash/power-cut reboots and Jetson stress tests. This package NEVER enables vehicle motion. Do not interpret offline test success as production sign-off.

**Deployment**: No automatic install/enable operation is shipped. Systemd service is deliberately not installable via `systemctl enable` without manual changes. Secret and SQLite state must live on a restricted ext4 Jetson directory, never committed to Git. To use local diagnostic endpoint for authorized bench work (on a disconnected rover), manually run `dyx_jetson_unix_probe --serve-unix /secure/owned/directory/probe.sock` with `DYX_HMAC_KEY_FILE` and `DYX_REPLAY_DB` configured.
