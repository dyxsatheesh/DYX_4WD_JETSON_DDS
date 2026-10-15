# Phase 2H5N — Fail-closed recovery-mode startup owner

**Prepared by Satheesh — DYX Autonomous**
**Repository:** `dyxsatheesh/DYX_4WD_JETSON_DDS`
**Branch:** `Jetson_DDS_Upgrade`
**Required baseline:** `69bb60d35aded76701a2849362f23a17b46f72cc`
**Scope:** OFFLINE C++17/SQLite validation only.

## Purpose

Prevent accidental motion-capable access through a newly constructed backend during a **recovery-mode** software restart. This patch creates an owner that constructs the gateway, mission-upload backend, and H5K boot guard internally, immediately latches the gateway, and performs exactly one H5J/H5H/H5I recovered PREVIEW review during construction. Its owned objects are private and cannot be obtained via getters.

The owner deliberately **never** grants upload, update, load, START, PAUSE, RESUME, NEXT or SKIP authority, even when a valid saved PREVIEW is found. Authorized GET `/api/v1/mission/draft` returns read-only review metadata after a fresh SQLite check. STOP can only yield a *software intent*, never a confirmed physical motor or PX4 stop. The durable H5H mutation fence remains closed.

## Changes

- `backend_recovery_startup_owner_lab.hpp` — private-owner, no-unquarantine offline C++17 boundary; quarantine latched before any caller has access; immutable startup admission; fail closed on invalid/missing staging and tamper.
- `draft_sqlite_recovery_startup_owner_native_smoke.cpp` — nine restart, quarantine, STOP, authorization, mutation, and corruption checks.
- `test/sqlite_lab/CMakeLists.txt` — nine new WSL2 SQLite CTest registrations.

## Test expectations

| Environment | Before | Added | After |
|---|---:|---:|---:|
| Windows native C++17 | 160 | 0 | 160 |
| Ubuntu WSL2 SQLite | 88 | 9 | **97** |
| Combined host suites | 248 | 9 | **257** |

## Limitations and deployment block

**NOT a production service or final integration.** This is specifically a quarantined-recovery-mode owner. Normal, non-recovery startup is not handled. It does not discover recovery mode, instantiate inside systemd, enforce every existing HTTP ingress path, verify identity/PX4 safety facts, synchronize multiple processes, or provide hardware isolation. A separately created gateway is not covered. The SQLite integrity checks are consistency checks, **not cryptographic authentication**. The offline C++17 API intentionally performs no network or DDS communication, no physical STOP, and no actuator control. A real production startup must exclusively own all motion-authority objects and gate every ingress path before any transport is opened. RC takeover and a physical emergency stop remain independent mandatory safeguards.

## Acceptance commands

In Windows PowerShell (Visual Studio 2022), after applying the patch, run native CMake build and full CTest. In **Ubuntu WSL2**, not Windows PowerShell, run SQLite CMake build and full CTest. Do not commit if either build or test fails.
