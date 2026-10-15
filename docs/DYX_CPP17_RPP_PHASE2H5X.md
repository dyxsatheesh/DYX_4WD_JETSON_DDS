# Phase 2H5X — Durable replay fence and single-process SQLite ownership
Prepared for DYX Autonomous. Adds `PersistentReplayFence` with SQLite `BEGIN IMMEDIATE`, strict increasing sequence per HMAC-authenticated session, `synchronous=FULL`, and a held `.owner` flock lock preventing parallel owners on the same database. Directory and file must be owned by effective UID without group/world permissions; no symlinks.

The local Unix HTTP probe now **refuses to start without `DYX_REPLAY_DB`** in addition to `DYX_HMAC_KEY_FILE`. Sequence is committed before returning HTTP 200. Replays or unavailable storage receive 409. No ROS, DDS, motor or spray command path is added. SQLite transaction is not a substitute for signed authorization or physical safety. The `.owner` lock is advisory within one Linux OS; shared/network filesystems and crash consistency must be rechecked on the Jetson NVMe.

Never place persistent replay DB on Windows `/mnt/d` for hardware deployment: use an owned local ext4 directory on the Jetson. Need host-side power-cut and fsync durability trials when hardware is available.
