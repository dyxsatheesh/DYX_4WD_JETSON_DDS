"""Per-second stream statistics for cmd_vel_bridge (ROS-free, testable).

Answers, from the Jetson itself rather than from a bag, where the setpoint
stream loses rate: RPP commands received vs setpoints published, which path
published them (RPP arrival, gate change, keep-alive), gaps in the RPP header
stamps the bridge actually received, RPP->bridge transport latency, gaps in
the bridge's own output and timer lateness.

Reading it: compare ``rpp_received`` per window with RPP's own cycle count
(``control_sequence`` in /rpp/debug, same bag). Fewer received than RPP ran
means commands were lost or coalesced before the bridge handled them. The
bag's recorded setpoint rate can then be checked against ``published``.
"""

from __future__ import annotations

import math
from typing import Any, Optional

PUBLISH_SOURCES = ("rpp", "gate_change", "keepalive")


def _percentile(values: list[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(fraction * len(ordered))) - 1))
    return ordered[index]


def _ms(value: Optional[float]) -> Optional[float]:
    return None if value is None else round(value * 1000.0, 2)


class BridgeStreamStats:
    def __init__(self, *, nominal_period_s: float, gap_threshold_s: float) -> None:
        self.nominal_period_s = float(nominal_period_s)
        self.gap_threshold_s = float(gap_threshold_s)
        self._last_rpp_stamp_s: Optional[float] = None
        self._last_publish_s: Optional[float] = None
        self._window_start_s: Optional[float] = None
        self._reset_window()

    def _reset_window(self) -> None:
        self.rpp_received = 0
        self.rpp_stamp_gaps = 0
        self.rpp_estimated_missing = 0
        self.rpp_max_stamp_gap_s = 0.0
        self.latencies_s: list[float] = []
        self.published = {source: 0 for source in PUBLISH_SOURCES}
        self.publish_gaps_over_threshold = 0
        self.max_publish_gap_s = 0.0
        self.max_timer_interval_s = 0.0
        self.late_timer_cycles = 0

    def on_rpp_command(self, now_s: float, stamp_s: Optional[float]) -> None:
        self.rpp_received += 1
        if stamp_s is None or stamp_s <= 0.0:
            return
        self.latencies_s.append(max(0.0, now_s - stamp_s))
        if self._last_rpp_stamp_s is not None:
            gap = stamp_s - self._last_rpp_stamp_s
            if gap > self.rpp_max_stamp_gap_s:
                self.rpp_max_stamp_gap_s = gap
            if gap > 1.5 * self.nominal_period_s:
                self.rpp_stamp_gaps += 1
                self.rpp_estimated_missing += max(
                    0, int(round(gap / self.nominal_period_s)) - 1
                )
        self._last_rpp_stamp_s = stamp_s

    def on_publish(self, now_s: float, source: str) -> None:
        self.published[source] = self.published.get(source, 0) + 1
        if self._last_publish_s is not None:
            gap = now_s - self._last_publish_s
            if gap > self.max_publish_gap_s:
                self.max_publish_gap_s = gap
            if gap > self.gap_threshold_s:
                self.publish_gaps_over_threshold += 1
        self._last_publish_s = now_s

    def on_timer(self, interval_s: float, *, late_threshold_s: float) -> None:
        if interval_s > self.max_timer_interval_s:
            self.max_timer_interval_s = interval_s
        if interval_s > late_threshold_s:
            self.late_timer_cycles += 1

    def snapshot(self, now_s: float) -> dict[str, Any]:
        """Return this window's statistics and start a new window."""
        start = self._window_start_s
        window_s = None if start is None else max(0.0, now_s - start)
        published_total = sum(self.published.values())
        payload = {
            "schema": "dyx4wd/cmd_vel_bridge_stats@1",
            "window_s": None if window_s is None else round(window_s, 3),
            "rpp_received": self.rpp_received,
            "rpp_stamp_gaps_over_1p5_periods": self.rpp_stamp_gaps,
            "rpp_estimated_missing": self.rpp_estimated_missing,
            "rpp_max_stamp_gap_ms": _ms(self.rpp_max_stamp_gap_s),
            "rpp_latency_ms": {
                "p50": _ms(_percentile(self.latencies_s, 0.50)),
                "p99": _ms(_percentile(self.latencies_s, 0.99)),
                "max": _ms(max(self.latencies_s) if self.latencies_s else None),
            },
            "published": published_total,
            "published_by_source": dict(self.published),
            "publish_rate_hz": (
                round(published_total / window_s, 2) if window_s else None
            ),
            "publish_gap_threshold_ms": _ms(self.gap_threshold_s),
            "publish_gaps_over_threshold": self.publish_gaps_over_threshold,
            "max_publish_gap_ms": _ms(self.max_publish_gap_s),
            "max_timer_interval_ms": _ms(self.max_timer_interval_s),
            "late_timer_cycles": self.late_timer_cycles,
        }
        self._window_start_s = now_s
        self._reset_window()
        return payload
