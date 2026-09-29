# Rover stack boot service (`dyx-rover.service`)

Added 2026-09-29. Starts `ros2 launch rover_bringup rover.launch.py` at every
Jetson boot, so the tablet app can connect without anyone SSHing in. It
launches the whole stack, including `rover_backend` on `:5001`.

| File | Role |
|---|---|
| `systemd/dyx-rover.service` | the unit (runs as `flash`) |
| `scripts/rover_launch.sh` | sources ROS + overlay, refuses a second stack, waits up to 60 s for the Pixhawk USB device, then `exec ros2 launch ...` |
| `scripts/install_rover_service.sh` | install / enable / remove (needs sudo) |

## Install (operator, once, on the Jetson)

`flash` has no passwordless sudo, so the operator runs this:

```bash
cd ~/rover_ws && git pull
sudo ./scripts/install_rover_service.sh --now
```

Without `--now` it is only enabled for the next boot. If a stack is already
running by hand, stop it first (Ctrl-C in its terminal): the service refuses to
start beside it (exit code 3, not retried).

## Day to day

```bash
systemctl status dyx-rover            # running?
journalctl -u dyx-rover -f            # live launch output
sudo systemctl restart dyx-rover      # restart the stack (e.g. after git pull)
sudo systemctl stop dyx-rover         # stop until next boot
sudo ./scripts/install_rover_service.sh --remove   # back to manual launch
```

⚠ **Do not also run `ros2 launch rover_bringup rover.launch.py` by hand while the
service is running.** Two stacks would both open `/dev/ttyACM0` and both try to
bind `:5001`. Stop the service first.

## Safety at boot

Starting at boot does not move the rover:
- `rover_backend` starts with `/emergency_stop=true`, `/mission_enable=false`
  and no mission prepared.
- Nothing arms PX4 or requests OFFBOARD until an operator STARTs a mission.
- RTK is restored from its persisted `desired_state` (as for a manual launch).

## Behaviour

- **Restart:** `on-failure`, 5 s apart, at most 5 times in 5 minutes. A node
  crash inside the launch does not restart the unit; `rover.launch.py` has its
  own per-node `respawn` settings.
- **Pixhawk missing:** waits 60 s (`DYX_FCU_WAIT_SEC`), then exits 4 and
  systemd retries.
- **Stop:** SIGINT to `ros2 launch`, which shuts the nodes down cleanly;
  anything left after 30 s is killed.
- **Independent of `bag-autorecord.service`**, which keeps running either way.

Not yet verified on the rover: first install and a real power-cycle boot.
