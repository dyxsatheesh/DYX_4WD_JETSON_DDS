"""cmd_vel_bridge stream statistics and the 100 Hz keep-alive check."""

import ast
from pathlib import Path
import sys

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from jetson_4wd_control.bridge_stats import BridgeStreamStats  # noqa: E402

BRIDGE = PACKAGE / "jetson_4wd_control" / "cmd_vel_bridge.py"


def stats():
    return BridgeStreamStats(nominal_period_s=0.020, gap_threshold_s=0.030)


def test_counts_received_published_by_source_and_rate():
    s = stats()
    s.snapshot(0.0)  # opens the window
    t = 0.0
    for i in range(50):
        t = 0.020 * (i + 1)
        s.on_rpp_command(t + 0.001, t)
        s.on_publish(t + 0.001, "rpp")
    s.on_publish(t + 0.026, "keepalive")
    out = s.snapshot(1.0)
    assert out["rpp_received"] == 50
    assert out["published"] == 51
    assert out["published_by_source"] == {"rpp": 50, "gate_change": 0, "keepalive": 1}
    assert out["publish_rate_hz"] == 51.0
    assert out["rpp_stamp_gaps_over_1p5_periods"] == 0
    assert out["rpp_latency_ms"]["p50"] == 1.0


def test_missing_rpp_cycles_show_as_stamp_gaps():
    s = stats()
    s.on_rpp_command(100.001, 100.000)
    s.on_rpp_command(100.021, 100.020)
    s.on_rpp_command(100.081, 100.080)  # .040 and .060 never arrived
    out = s.snapshot(1.0)
    assert out["rpp_stamp_gaps_over_1p5_periods"] == 1
    assert out["rpp_estimated_missing"] == 2
    assert out["rpp_max_stamp_gap_ms"] == 60.0


def test_output_gaps_and_timer_lateness():
    s = stats()
    s.on_publish(0.000, "rpp")
    s.on_publish(0.020, "rpp")
    s.on_publish(0.352, "keepalive")  # the 332 ms stall class
    s.on_timer(0.010, late_threshold_s=0.040)
    s.on_timer(0.330, late_threshold_s=0.040)
    out = s.snapshot(1.0)
    assert out["publish_gaps_over_threshold"] == 1
    assert out["max_publish_gap_ms"] == 332.0
    assert out["late_timer_cycles"] == 1
    assert out["max_timer_interval_ms"] == 330.0


def test_snapshot_starts_a_fresh_window_but_keeps_continuity():
    s = stats()
    s.on_rpp_command(100.001, 100.000)
    s.on_publish(100.001, "rpp")
    s.snapshot(101.0)
    s.on_rpp_command(101.001, 101.000)  # 1 s after the last stamp: a real gap
    s.on_publish(101.001, "rpp")
    out = s.snapshot(102.0)
    assert out["rpp_received"] == 1 and out["published"] == 1
    assert out["rpp_stamp_gaps_over_1p5_periods"] == 1
    assert out["window_s"] == 1.0


def test_empty_window_is_serialisable():
    out = stats().snapshot(1.0)
    assert out["rpp_latency_ms"] == {"p50": None, "p99": None, "max": None}
    assert out["publish_rate_hz"] is None


def _class_source():
    source = BRIDGE.read_text(encoding="utf-8")
    return source, ast.parse(source)


def test_bridge_checks_at_100_hz_and_keeps_the_25_ms_keepalive():
    source, _tree = _class_source()
    assert "CHECK_HZ = 100.0" in source
    assert "STREAM_HZ = 50.0" in source
    assert "KEEPALIVE_PERIODS = 1.25" in source
    assert "1.0 / self.CHECK_HZ,\n            self._control_loop," in source


def test_bridge_counts_every_rpp_message_and_publishes_stats():
    source, tree = _class_source()
    for name in ("_rpp_callback", "_rpp_command_callback"):
        node = next(
            n for n in ast.walk(tree)
            if isinstance(n, ast.FunctionDef) and n.name == name
        )
        first = node.body[0]
        assert "self._note_rpp_command(message)" in ast.get_source_segment(source, first)
    assert '"/cmd_vel_bridge/stats"' in source
    assert "self._stats.on_publish(" in source


def test_unset_zero_stamp_is_counted_but_not_used_for_gaps_or_latency():
    s = stats()
    s.on_rpp_command(100.0, 0.0)
    out = s.snapshot(101.0)
    assert out["rpp_received"] == 1
    assert out["rpp_latency_ms"]["max"] is None
