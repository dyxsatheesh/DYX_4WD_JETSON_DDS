"""A21 frame parsing, zone tracking and settings (pure, no ROS needed)."""

from __future__ import annotations

import json
import sys
import types
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from obstacle_monitor.a21_protocol import (  # noqa: E402
    KIND_BLIND_ZONE,
    KIND_DISTANCE,
    KIND_INTERFERENCE,
    KIND_NO_ECHO,
    FrameParser,
    classify,
)
from obstacle_monitor.zones import (  # noqa: E402
    ZONE_CLEAR,
    ZONE_DANGER,
    ZONE_UNKNOWN,
    ZONE_WARNING,
    ZoneConfig,
    ZoneTracker,
)


def frame(mm: int) -> bytes:
    high, low = mm >> 8, mm & 0xFF
    return bytes([0xFF, high, low, (0xFF + high + low) & 0xFF])


# ------------------------------------------------------------ protocol


def test_parses_frames_split_across_reads_and_skips_garbage():
    parser = FrameParser()
    stream = b"\x12\x34" + frame(1234) + frame(56)
    readings = parser.feed(stream[:5]) + parser.feed(stream[5:])
    assert [(r.kind, r.distance_mm) for r in readings] == [
        (KIND_DISTANCE, 1234),
        (KIND_DISTANCE, 56),
    ]


def test_bad_checksum_resyncs():
    parser = FrameParser()
    bad = bytes([0xFF, 0x04, 0xD2, 0x00])
    readings = parser.feed(bad + frame(900))
    assert [r.distance_mm for r in readings] == [900]
    assert parser.bad_checksums == 1


@pytest.mark.parametrize(
    "raw,kind",
    [(0xFFFE, KIND_INTERFERENCE), (0xFFFD, KIND_NO_ECHO), (0xFFFF, KIND_NO_ECHO),
     (0, KIND_BLIND_ZONE), (2500, KIND_DISTANCE)],
)
def test_special_values(raw, kind):
    assert classify(raw).kind == kind


# ------------------------------------------------------------ zones


def feed(tracker, mm_values, *, start=0.0, step=0.1):
    t = start
    changes = []
    for mm in mm_values:
        if tracker.update(classify(mm), t):
            changes.append(tracker.confirmed_zone)
        t += step
    return changes, t


def test_zone_needs_three_consecutive_readings():
    tracker = ZoneTracker(ZoneConfig(danger_mm=1500, warning_mm=3000))
    changes, _ = feed(tracker, [1000, 1000, 4000, 1000, 1000])
    assert changes == []
    changes, _ = feed(tracker, [1000])
    assert changes == [ZONE_DANGER]


def test_zones_by_distance():
    tracker = ZoneTracker(ZoneConfig(danger_mm=1500, warning_mm=3000))
    assert feed(tracker, [2500] * 3)[0] == [ZONE_WARNING]
    assert feed(tracker, [1500] * 3)[0] == [ZONE_DANGER]


def test_leaving_danger_needs_hysteresis():
    tracker = ZoneTracker(ZoneConfig(danger_mm=1500, warning_mm=3000, hysteresis_mm=200))
    feed(tracker, [1000] * 3)
    assert feed(tracker, [1650] * 5)[0] == []          # still within 1500+200
    assert feed(tracker, [1750] * 3)[0] == [ZONE_WARNING]


def test_no_echo_is_clear_and_blind_zone_is_danger():
    tracker = ZoneTracker(ZoneConfig())
    assert feed(tracker, [0] * 3)[0] == [ZONE_DANGER]
    assert feed(tracker, [0xFFFD] * 3)[0] == [ZONE_CLEAR]


def test_interference_keeps_sensor_alive_but_does_not_change_zone():
    tracker = ZoneTracker(ZoneConfig())
    feed(tracker, [1000] * 3)
    changes, t = feed(tracker, [0xFFFE] * 5, start=1.0)
    assert changes == [] and tracker.confirmed_zone == ZONE_DANGER
    assert tracker.healthy(t)


def test_health_timeout_makes_zone_unknown():
    tracker = ZoneTracker(ZoneConfig(health_timeout_sec=0.5))
    assert tracker.zone(0.0) == ZONE_UNKNOWN          # never heard from
    feed(tracker, [1000] * 3)
    assert tracker.zone(0.3) == ZONE_DANGER
    assert tracker.zone(5.0) == ZONE_UNKNOWN


@pytest.mark.parametrize("danger,warning", [(1500, 1500), (200, 3000), (1500, 8000), (3000, 1500)])
def test_invalid_zone_config_rejected(danger, warning):
    with pytest.raises(ValueError):
        ZoneConfig(danger_mm=danger, warning_mm=warning).validate()


def test_reconfigure_applies_immediately():
    tracker = ZoneTracker(ZoneConfig(danger_mm=1500, warning_mm=3000))
    feed(tracker, [2000] * 3)
    assert tracker.confirmed_zone == ZONE_WARNING
    tracker.reconfigure(ZoneConfig(danger_mm=2500, warning_mm=4000))
    assert feed(tracker, [2000] * 3)[0] == [ZONE_DANGER]


# ------------------------------------------------------------ settings

def _node_module():
    """Import the node module with ROS stubbed when rclpy is absent."""
    try:
        import rclpy  # noqa: F401
    except ImportError:
        class _Any:
            def __init__(self, *a, **k):
                pass

            def __getattr__(self, name):
                return _Any()

            def __call__(self, *a, **k):
                return _Any()

        for name, attrs in {
            "rclpy": {"ok": lambda: True, "init": lambda *a, **k: None},
            "rclpy.node": {"Node": type("Node", (), {})},
            "rclpy.qos": {"DurabilityPolicy": _Any(), "HistoryPolicy": _Any(),
                          "QoSProfile": _Any, "ReliabilityPolicy": _Any()},
            "std_msgs": {},
            "std_msgs.msg": {"String": _Any},
            "sensor_msgs": {},
            "sensor_msgs.msg": {"Range": _Any},
        }.items():
            module = types.ModuleType(name)
            module.__dict__.update(attrs)
            sys.modules.setdefault(name, module)
    import importlib

    return importlib.import_module("obstacle_monitor.obstacle_monitor_node")


def test_settings_round_trip_and_bad_file_falls_back(tmp_path):
    node = _node_module()
    path = tmp_path / "obstacle.json"
    defaults = node.ObstacleSettings()
    assert node.load_settings(path, defaults) == defaults

    saved = node.ObstacleSettings(enabled=True, danger_mm=1200, warning_mm=2500)
    node.save_settings(path, saved)
    assert node.load_settings(path, defaults) == saved

    path.write_text(json.dumps({"danger_mm": 5000, "warning_mm": 1000}))
    assert node.load_settings(path, defaults) == defaults


def test_config_request_validation():
    node = _node_module()
    current = node.ObstacleSettings()
    updated = node.apply_config_request(current, {"enabled": True, "danger_mm": 1200})
    assert updated.enabled is True and updated.danger_mm == 1200 and updated.warning_mm == 3000

    for bad in (
        {"enabled": "yes"},
        {"danger_mm": 3500},
        {"warning_mm": "far"},
        {"port": "COM3"},
        {"baud": 57600},
    ):
        with pytest.raises(ValueError):
            node.apply_config_request(current, bad)


def test_range_values_follow_rep_117():
    import math

    node = _node_module()
    assert node.range_value_m(classify(1234)) == pytest.approx(1.234)
    assert node.range_value_m(classify(0xFFFD)) == math.inf     # nothing in range
    assert node.range_value_m(classify(0)) == -math.inf         # blind zone: too close


def test_bags_record_the_obstacle_topics():
    source = (PACKAGE_ROOT.parents[1] / "scripts" / "bag_autorecord.py").read_text()
    for topic in ("/obstacle/status", "/obstacle/range", "/obstacle/config"):
        assert f'"{topic}"' in source
    assert '"/obstacle_monitor"' in source          # node params saved with each bag


# ------------------------------------------------------------ silent-port watchdog


class _FakeClock:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t


def _reader_under_test(node, reads_per_open, clock, max_opens, monkeypatch):
    """A node object with only what _read_loop needs, and a fake serial port."""
    import threading

    opened = []

    class FakeSerial:
        def __init__(self, port, baud, timeout):
            opened.append(port)
            self._reads = list(reads_per_open[min(len(opened), len(reads_per_open)) - 1])
            if len(opened) >= max_opens:
                reader._stop.set()

        def read(self, size):
            clock.t += 0.1  # each read blocks for the 0.1 s serial timeout
            return self._reads.pop(0) if self._reads else b""

        def close(self):
            pass

    logs = []

    class Logger:
        def warn(self, text):
            logs.append(("warn", text))

        def error(self, text):
            logs.append(("error", text))

    reader = node.ObstacleMonitorNode.__new__(node.ObstacleMonitorNode)
    reader._lock = threading.Lock()
    reader._settings = node.ObstacleSettings(port="/dev/fake", baud=115200)
    reader._stop = threading.Event()
    reader._reopen = threading.Event()
    reader._serial_error = None
    reader._port_open = False
    received = []
    reader._on_bytes = received.append
    reader.get_logger = lambda: Logger()
    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=FakeSerial))
    return reader, opened, received, logs


def test_silent_port_is_reopened_and_reported_once(monkeypatch):
    node = _node_module()
    clock = _FakeClock()
    monkeypatch.setattr(node.time, "monotonic", clock.monotonic)
    # First open streams one frame and then goes silent (the USB stall);
    # every later open stays silent.
    reader, opened, received, logs = _reader_under_test(
        node, [[frame(1000)], []], clock, max_opens=4, monkeypatch=monkeypatch
    )

    reader._read_loop()

    assert len(opened) == 4                      # reopened after each silence
    assert received == [frame(1000)]
    errors = [text for level, text in logs if level == "error"]
    assert len(errors) == 1                      # one log line, not one per retry
    assert errors[0].startswith("Obstacle sensor: no data from /dev/fake")
    assert reader._serial_error.startswith(node.SILENCE_ERROR_PREFIX)


def test_data_after_reopen_clears_the_silence_error(monkeypatch):
    node = _node_module()
    clock = _FakeClock()
    monkeypatch.setattr(node.time, "monotonic", clock.monotonic)
    reader, opened, received, logs = _reader_under_test(
        node, [[], [frame(800)]], clock, max_opens=3, monkeypatch=monkeypatch
    )

    reader._read_loop()

    assert frame(800) in received
    assert any("data resumed" in text for level, text in logs if level == "warn")


def test_silence_threshold():
    node = _node_module()
    limit = node.ObstacleMonitorNode.SILENCE_REOPEN_SEC
    assert not node.silence_exceeded(10.0, 10.0 + limit, limit)
    assert node.silence_exceeded(10.0, 10.0 + limit + 0.1, limit)
