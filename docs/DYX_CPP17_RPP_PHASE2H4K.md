# DYX Autonomous — Phase 2H4K: Offline HTTP Listener Deadline Hardening

**Prepared by Satheesh**
**Base:** Phase 2H4J (`93e285e`), `Jetson_DDS_Upgrade`.
**Target:** C++17 MSVC/Windows and GCC/Linux without ROS 2, Jetson or hardware.

## Objective

Harden the existing `127.0.0.1` developer-only read-only HTTP server against an incomplete or slow-trickling local client that otherwise occupied the single event loop indefinitely. No network mission-changing commands are enabled.

## Changes

- Total *monotonic* header-read deadline per accepted connection; individual recv activity does not extend the deadline. Expired headers return HTTP `408 Request Timeout`.
- Strictly bounded accumulated header bytes (up to 4096, configurable downward). Oversize requests return HTTP `413` before dispatch. Disconnects with incomplete headers return `400`.
- Bounded poll intervals and total response-send deadline; a stalled receiver cannot trap the event loop indefinitely.
- Interruption on listener `stop()` even while one slow/incomplete client is connected; subsequent listener restart works.
- POSIX `MSG_NOSIGNAL` for socket sends on platforms that support it; closed peer cannot terminate the listener with SIGPIPE.
- Fixed defaults: 4096-byte headers, 1200 ms total header deadline, 1200 ms total response deadline, 50 ms I/O poll. Configurable only within validated limits.
- Four new real TCP loopback tests: slow trickle, early disconnect, oversized/pipelined bytes, shutdown/restart with an incomplete request.

## Safety and scope

The HTTP listener remains bound **only** to IPv4 `127.0.0.1`, supports **GET only**, and uses an artificial local test principal. It has **no TLS, real authentication, Android remote access, PX4/DDS connection or actuator control**. Header/body defenses in this patch are not a substitute for a proper production HTTP stack and threat review.

The four tests use local sockets only. Concurrent application mutation of the injected mission gateway remains prohibited: the loopback listener serves one client at a time and is **not** a production multithreaded server. A queued connection in the OS accept backlog is not processed until the active connection exits; short deadlines bound that exposure but do not provide load shedding/rate limiting.

## Offline verification

```powershell
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G "Visual Studio 17 2022" -A x64
cmake --build build/dyx_cpp_only --config Release
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
```

Expected: **96 native tests (92 previous + 4 new)**. Field verification remains deferred.
