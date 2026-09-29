#!/usr/bin/env python3
"""End-to-end check of the obstacle_monitor node with simulated A21 frames.

Safe to run while the rover stack is up: it starts a SEPARATE test instance
(node obstacle_monitor_test, topics under /obstacle_test/, its own settings
file in /tmp), so mission_manager and cmd_vel_bridge never see its data.

    source /opt/ros/humble/setup.bash && source ~/rover_ws/install/setup.bash
    python3 ~/rover_ws/scripts/test_obstacle_pipeline.py

A virtual serial port (pty) stands in for the sensor. The script writes real
A21 UART frames (FF H L SUM) through these phases and checks what the node
publishes:

    clear   (no echo)        -> zone clear,   /range = +inf
    warning (2500 mm)        -> zone warning, /range = 2.5 m
    danger  (1000 mm)        -> zone danger,  /range = 1.0 m
    silent  (no frames 1.5s) -> zone unknown, healthy false (sensor lost)
    clear   again            -> zone clear,   healthy true

Exit code 0 = PASS.
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String

import json

FRAME_PERIOD_SEC = 0.1  # A21 automatic mode streams about 10 Hz


def frame(mm: int) -> bytes:
    high, low = mm >> 8, mm & 0xFF
    return bytes([0xFF, high, low, (0xFF + high + low) & 0xFF])


class Collector(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_pipeline_test_collector")
        self.statuses: list[tuple[float, dict]] = []
        self.ranges: list[tuple[float, float]] = []
        self.create_subscription(String, "/obstacle_test/status", self._status, 10)
        self.create_subscription(Range, "/obstacle_test/range", self._range, 10)

    def _status(self, message: String) -> None:
        self.statuses.append((time.monotonic(), json.loads(message.data)))

    def _range(self, message: Range) -> None:
        self.ranges.append((time.monotonic(), message.range))


def spin_for(node: Node, seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def feed(node: Node, master: int, mm: int, seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        os.write(master, frame(mm))
        spin_for(node, FRAME_PERIOD_SEC)


def main() -> int:
    master, slave = os.openpty()
    port = os.ttyname(slave)
    settings = "/tmp/obstacle_pipeline_test_settings.json"
    if os.path.exists(settings):
        os.unlink(settings)

    test_node = subprocess.Popen(
        [
            "ros2", "run", "obstacle_monitor", "obstacle_monitor_node",
            "--ros-args",
            "-r", "__node:=obstacle_monitor_test",
            "-r", "/obstacle/status:=/obstacle_test/status",
            "-r", "/obstacle/range:=/obstacle_test/range",
            "-r", "/obstacle/config:=/obstacle_test/config",
            "-p", f"port:={port}",
            "-p", "enabled:=true",
            "-p", f"settings_path:={settings}",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )

    rclpy.init()
    collector = Collector()
    results: list[tuple[str, bool, str]] = []
    try:
        spin_for(collector, 4.0)  # node start-up + serial open
        if not collector.statuses:
            print("FAIL: no /obstacle_test/status from the test node")
            return 1

        phases = [("clear", 0xFFFD, 1.5), ("warning", 2500, 1.5), ("danger", 1000, 1.5)]
        for expected, mm, seconds in phases:
            start = time.monotonic()
            feed(collector, master, mm, seconds)
            status = collector.statuses[-1][1]
            ranges = [value for t, value in collector.ranges if t >= start]
            want_range = math.inf if mm == 0xFFFD else mm / 1000.0
            ok = (
                status.get("zone") == expected
                and status.get("healthy") is True
                and bool(ranges)
                and all(v == want_range for v in ranges[-3:])
            )
            results.append((
                f"{expected:<8} ({'no echo' if mm == 0xFFFD else f'{mm} mm'})",
                ok,
                f"zone={status.get('zone')} healthy={status.get('healthy')} "
                f"distance_mm={status.get('distance_mm')} range_msgs={len(ranges)} "
                f"last_range={ranges[-1] if ranges else None}",
            ))

        spin_for(collector, 1.5)  # sensor goes silent
        status = collector.statuses[-1][1]
        results.append((
            "silent   (no frames)",
            status.get("zone") == "unknown" and status.get("healthy") is False,
            f"zone={status.get('zone')} healthy={status.get('healthy')}",
        ))

        feed(collector, master, 0xFFFD, 1.5)
        status = collector.statuses[-1][1]
        results.append((
            "recover  (no echo)",
            status.get("zone") == "clear" and status.get("healthy") is True,
            f"zone={status.get('zone')} healthy={status.get('healthy')}",
        ))

        window = [t for t, _ in collector.statuses if t >= collector.statuses[-1][0] - 2.0]
        rate = (len(window) - 1) / (window[-1] - window[0]) if len(window) > 1 else 0.0
        results.append(("status rate", rate >= 8.0, f"{rate:.1f} Hz (want ~10+)"))
    finally:
        collector.destroy_node()
        rclpy.shutdown()
        test_node.terminate()
        try:
            test_node.wait(timeout=5)
        except subprocess.TimeoutExpired:
            test_node.kill()
        os.close(master)
        os.close(slave)

    width = max(len(name) for name, _, _ in results)
    for name, ok, detail in results:
        print(f"{'PASS' if ok else 'FAIL'}  {name:<{width}}  {detail}")
    passed = all(ok for _, ok, _ in results)
    print("\nRESULT:", "PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
