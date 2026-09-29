"""Forward ultrasonic obstacle monitor (stop and wait).

Ported from the 3WD NRP_ROS ``Backend/obstacle_monitor.py`` (DYP A21 on a
Jetson UART). This node only *measures and classifies*; it never commands
motion. Consumers:

- mission_manager pauses a RUNNING mission on ``danger`` (manual resume only)
  and refuses START/RESUME/NEXT while danger persists;
- cmd_vel_bridge independently holds zero velocity while ``danger``;
- rover_backend shows it and speaks warnings.

A sensor failure is reported (``healthy: false``, zone ``unknown``) but is a
*warning only*: nobody pauses on it (operator decision 2026-09-29).

Publishes ``/obstacle/status`` (std_msgs/String JSON, ``dyx4wd/obstacle@1``)
at 10 Hz and immediately on a zone change. Settings (enabled, zones, port,
baud) persist in ``settings_path`` and change through ``/obstacle/config``
(JSON with request_id; the result is echoed in the status).
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from obstacle_monitor.a21_protocol import FrameParser
from obstacle_monitor.zones import ZoneConfig, ZoneTracker

SCHEMA = "dyx4wd/obstacle@1"


@dataclass(frozen=True)
class ObstacleSettings:
    enabled: bool = False
    danger_mm: int = 1500
    warning_mm: int = 3000
    port: str = "/dev/ttyTHS1"
    baud: int = 115200

    def zone_config(self, base: ZoneConfig) -> ZoneConfig:
        return replace(base, danger_mm=self.danger_mm, warning_mm=self.warning_mm)


def load_settings(path: Path, defaults: ObstacleSettings) -> ObstacleSettings:
    """Saved settings over the launch defaults; bad files fall back."""
    try:
        if not path.exists():
            return defaults
        payload = json.loads(path.read_text(encoding="utf-8"))
        merged = {**asdict(defaults), **{k: payload[k] for k in asdict(defaults) if k in payload}}
        settings = ObstacleSettings(
            enabled=bool(merged["enabled"]),
            danger_mm=int(merged["danger_mm"]),
            warning_mm=int(merged["warning_mm"]),
            port=str(merged["port"]),
            baud=int(merged["baud"]),
        )
        settings.zone_config(ZoneConfig()).validate()
        return settings
    except Exception:
        return defaults


def save_settings(path: Path, settings: ObstacleSettings) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps(asdict(settings), sort_keys=True))
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def apply_config_request(
    current: ObstacleSettings, payload: dict[str, Any]
) -> ObstacleSettings:
    """Merge a /obstacle/config request into the settings; raises ValueError."""
    changes: dict[str, Any] = {}
    if "enabled" in payload:
        if not isinstance(payload["enabled"], bool):
            raise ValueError("enabled must be true or false")
        changes["enabled"] = payload["enabled"]
    for key in ("danger_mm", "warning_mm", "baud"):
        if key in payload:
            value = payload[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{key} must be a number")
            changes[key] = int(value)
    if "port" in payload:
        port = str(payload["port"]).strip()
        if not port.startswith("/dev/"):
            raise ValueError("port must be a /dev/... path")
        changes["port"] = port
    if "baud" in changes and changes["baud"] not in (9600, 115200):
        raise ValueError("baud must be 9600 or 115200")
    candidate = replace(current, **changes)
    candidate.zone_config(ZoneConfig()).validate()
    return candidate


class ObstacleMonitorNode(Node):
    PUBLISH_HZ = 10.0
    REOPEN_INTERVAL_SEC = 2.0

    def __init__(self) -> None:
        super().__init__("obstacle_monitor")

        self.declare_parameter("enabled", False)
        self.declare_parameter("danger_mm", 1500)
        self.declare_parameter("warning_mm", 3000)
        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("debounce_count", 3)
        self.declare_parameter("hysteresis_mm", 200)
        self.declare_parameter("health_timeout_sec", 0.5)
        self.declare_parameter("settings_path", "~/.ros/dyx_obstacle_settings.json")

        launch_defaults = ObstacleSettings(
            enabled=bool(self.get_parameter("enabled").value),
            danger_mm=int(self.get_parameter("danger_mm").value),
            warning_mm=int(self.get_parameter("warning_mm").value),
            port=str(self.get_parameter("port").value),
            baud=int(self.get_parameter("baud").value),
        )
        self._settings_path = Path(
            os.path.expanduser(str(self.get_parameter("settings_path").value))
        )
        self._settings = load_settings(self._settings_path, launch_defaults)
        self._settings_source = "saved" if self._settings_path.exists() else "launch"
        self._base_zone_config = ZoneConfig(
            debounce_count=int(self.get_parameter("debounce_count").value),
            hysteresis_mm=int(self.get_parameter("hysteresis_mm").value),
            health_timeout_sec=float(self.get_parameter("health_timeout_sec").value),
        )

        self._lock = threading.Lock()
        self._tracker = ZoneTracker(self._settings.zone_config(self._base_zone_config))
        self._parser = FrameParser()
        self._serial_error: Optional[str] = None
        self._port_open = False
        self._reopen = threading.Event()
        self._stop = threading.Event()
        self._last_published_zone: Optional[str] = None
        self._config_request_id: Optional[str] = None
        self._config_result: Optional[str] = None
        self._config_reason: Optional[str] = None

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._status_pub = self.create_publisher(String, "/obstacle/status", status_qos)
        self.create_subscription(String, "/obstacle/config", self._config_callback, status_qos)
        self.create_timer(1.0 / self.PUBLISH_HZ, self._publish_status)

        self._reader = threading.Thread(target=self._read_loop, name="a21-reader", daemon=True)
        self._reader.start()

        self.get_logger().warn(
            f"Obstacle monitor: enabled={self._settings.enabled} "
            f"danger<={self._settings.danger_mm}mm warning<={self._settings.warning_mm}mm "
            f"port={self._settings.port}@{self._settings.baud} "
            f"(settings {self._settings_source}: {self._settings_path})"
        )

    # ------------------------------------------------------------ serial

    def _read_loop(self) -> None:
        import serial  # python3-serial

        while not self._stop.is_set():
            with self._lock:
                port, baud = self._settings.port, self._settings.baud
            try:
                handle = serial.Serial(port, baud, timeout=0.1)
            except Exception as exc:
                self._set_serial_error(f"cannot open {port}: {exc}")
                self._stop.wait(self.REOPEN_INTERVAL_SEC)
                continue
            self._set_serial_error(None, port_open=True)
            self._reopen.clear()
            try:
                while not self._stop.is_set() and not self._reopen.is_set():
                    data = handle.read(64)
                    if data:
                        self._on_bytes(data)
            except Exception as exc:
                self._set_serial_error(f"read error on {port}: {exc}")
                self._stop.wait(self.REOPEN_INTERVAL_SEC)
            finally:
                try:
                    handle.close()
                except Exception:
                    pass
                with self._lock:
                    self._port_open = False

    def _set_serial_error(self, error: Optional[str], *, port_open: bool = False) -> None:
        with self._lock:
            changed = error != self._serial_error
            self._serial_error = error
            self._port_open = port_open
        if changed and error:
            self.get_logger().error(f"Obstacle sensor: {error}")

    def _on_bytes(self, data: bytes) -> None:
        now = time.monotonic()
        zone_changed = False
        with self._lock:
            for reading in self._parser.feed(data):
                zone_changed |= self._tracker.update(reading, now)
        if zone_changed:
            self._publish_status()

    # ------------------------------------------------------------ config

    def _config_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("config must be a JSON object")
        except Exception as exc:
            self.get_logger().error(f"Rejected obstacle config: {exc}")
            return
        request_id = str(payload.get("request_id") or "").strip()
        try:
            with self._lock:
                candidate = apply_config_request(self._settings, payload)
            save_settings(self._settings_path, candidate)
        except Exception as exc:
            with self._lock:
                self._config_request_id = request_id
                self._config_result = "REJECTED"
                self._config_reason = str(exc)
            self._publish_status()
            return
        with self._lock:
            port_changed = (candidate.port, candidate.baud) != (
                self._settings.port,
                self._settings.baud,
            )
            self._settings = candidate
            self._settings_source = "saved"
            self._tracker.reconfigure(candidate.zone_config(self._base_zone_config))
            self._config_request_id = request_id
            self._config_result = "ACCEPTED"
            self._config_reason = None
        if port_changed:
            self._reopen.set()
        self.get_logger().warn(
            f"Obstacle config: enabled={candidate.enabled} danger<={candidate.danger_mm}mm "
            f"warning<={candidate.warning_mm}mm port={candidate.port}@{candidate.baud}"
        )
        self._publish_status()

    # ------------------------------------------------------------ status

    def status_payload(self, now: float) -> dict[str, Any]:
        with self._lock:
            tracker = self._tracker
            healthy = tracker.healthy(now)
            age = None if tracker.last_reading_at is None else round(now - tracker.last_reading_at, 3)
            return {
                "schema": SCHEMA,
                "enabled": self._settings.enabled,
                "healthy": healthy,
                "zone": tracker.zone(now),
                "distance_mm": tracker.distance_mm if healthy else None,
                "last_reading_age_sec": age,
                "danger_mm": self._settings.danger_mm,
                "warning_mm": self._settings.warning_mm,
                "port": self._settings.port,
                "baud": self._settings.baud,
                "port_open": self._port_open,
                "serial_error": self._serial_error,
                "readings": tracker.readings,
                "bad_checksums": self._parser.bad_checksums,
                "settings_source": self._settings_source,
                "config_request_id": self._config_request_id,
                "config_result": self._config_result,
                "config_reason": self._config_reason,
                "timestamp_unix_ns": time.time_ns(),
            }

    def _publish_status(self) -> None:
        payload = self.status_payload(time.monotonic())
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self._status_pub.publish(message)
        if payload["zone"] != self._last_published_zone:
            if self._last_published_zone is not None and payload["enabled"]:
                self.get_logger().warn(
                    f"Obstacle zone {self._last_published_zone} -> {payload['zone']} "
                    f"(distance={payload['distance_mm']}mm)"
                )
            self._last_published_zone = payload["zone"]

    def shutdown(self) -> None:
        self._stop.set()
        self._reopen.set()
        self._reader.join(timeout=1.0)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ObstacleMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
