# C1: NTRIP receive + RTCM3 validation (C++17)

A native ROS 2 C++17 executable `rtk_correction_bridge_node` subscribes to no rover control topics and publishes only `/rtk_correction_bridge/status`.

Supports opt-in NTRIP TCP/TLS receive, Basic auth, HTTP/ICY 200 header validation, bounded incremental RTCM3 parser and CRC-24Q validation, reconnect after disconnect, and default-disabled network activity. TLS is required by default with CA and hostname checks; plaintext requires deliberate opt-out.

**Important: NOT receiver injection:** No USB/serial output, no GGA uplink, no RTK acceptance claim, no correction-age trust signal, no PX4-in commands. Production direct RTCM serial sink, exclusive lock, NMEA GGA, receiver verified correction age, TLS/RTCM caster integration tests, and lifecycle tests remain pending. No motion or spray authorization.

Do not enable `receive_enabled` on deployment without reviewing NTRIP caster network credentials, mountpoint and data governance. Password supplied as a ROS parameter may be visible through ROS parameter interfaces, shell history or process arguments; prefer protected configuration and credentials management before production. This C1 implementation is a candidate requiring integration verification.
