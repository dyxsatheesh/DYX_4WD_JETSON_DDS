# Phase 2H5W — Local authenticated diagnostic HTTP only
Prepared for DYX Autonomous. Linux AF_UNIX socket implementation, **no TCP/remote Android endpoint**, no PX4/DDS outputs.

`dyx_jetson_unix_probe --serve-unix /secure/owned-directory/probe.sock` requires `DYX_HMAC_KEY_FILE` to name a pre-provisioned, strictly owner-only 32–256-byte secret. For laboratory setup, provision out of repository with `umask 077; openssl rand -out /private/secret/path 32`. NEVER commit secrets.

Socket parent must already exist, be owned by the service UID and mode 0700; socket cannot preexist. Peer UID must match service UID. Request size capped at 2048 bytes, timeout 500ms, method/path exactly `GET /health`, duplicate headers/body/Transfer-Encoding rejected. Requires signed session, sequence, nonce; HMAC-SHA256 over the canonical fields. Only returns software inhibition status; 200 response **does not attest physical stop**. Signatures remain replayable until the H5X transaction fence is integrated. Do not bind a TCP proxy to this socket or expose via Android.

A separate production TLS identity provider, rate limiting and end-to-end operator access control remain to be designed and hardware-tested.
