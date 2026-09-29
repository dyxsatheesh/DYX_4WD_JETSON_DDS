"""Which rover events are spoken, and when (important events only).

Edge-triggered from what the backend already receives: mission_manager
status (state / pause reason / active point), point events, GPS fix type,
battery percentage and the MAVROS link. Pure logic around a ``say`` sink so
it is unit-testable without ROS or audio. Every entry point swallows its own
errors: announcing must never break a ROS callback.

Spoken:
- mission ready / started / paused (with reason) / resumed / stopped /
  completed / error, and Emergency stop (pause reason ESTOP);
- "Heading to target N" whenever the rover sets off for a point;
- per point: "Target N done" (SPRAY), "Target N. Laser on. Mark here."
  (LASER), "... Outside tolerance. Do not mark." (LASER measured miss),
  "Target N failed" / "Target N skipped";
- RTK fixed / float / lost (12 s cooldown; silent while RUNNING/PAUSED,
  where the pause reason is already announced);
- battery low <= 20 %, critical <= 10 % (once each, re-armed above 25 %);
- vehicle link lost during an active mission, and restored afterwards;
- obstacle: "Obstacle ahead, N centimetres" on entering the warning zone
  while RUNNING (10 s cooldown), "Obstacle. Mission paused." (pause reason
  OBSTACLE), and "Obstacle sensor lost" once when an enabled sensor stops
  reporting during a mission (warning only; the mission continues);
- a greeting when the vehicle first connects.
"""

from __future__ import annotations

import logging
import threading
import time
from datetime import datetime
from typing import Any, Callable, Optional

from rover_backend import voice_phrases as phrases

LOGGER = logging.getLogger(__name__)

ACTIVE_STATES = frozenset({"RUNNING", "PAUSED", "WAITING_FOR_NEXT"})
RTK_QUIET_STATES = frozenset({"RUNNING", "PAUSED"})

PAUSE_REASON_PHRASES = {
    "ESTOP": "emergency_stop",
    "RTK_FLOAT": "paused_rtk_float",
    "RTK_FLOAT_TIMEOUT": "paused_rtk_float",
    "RTK_LOST": "paused_rtk_lost",
    "PX4_CONTROL_LOST": "paused_vehicle_control",
    "ODOM_STALE": "paused_position",
    "OBSTACLE": "obstacle_paused",
}
PRIORITY_PHRASES = frozenset(
    {"emergency_stop", "connection_lost", *PAUSE_REASON_PHRASES.values(),
     "mission_paused", "mission_error", "battery_critical"}
)

LASER_MEASURED_MISS_PREFIX = "RAW_GNSS_RADIAL_EXCEEDS_TOLERANCE"
RTK_ANNOUNCE_COOLDOWN_SEC = 12.0
OBSTACLE_WARNING_COOLDOWN_SEC = 10.0

Say = Callable[..., None]


def _state(value: Any) -> str:
    return str(value or "").strip().upper()


class VoiceAnnouncer:
    def __init__(
        self,
        say: Say,
        *,
        precache: Optional[Callable[[list[tuple[str, dict[str, Any]]]], Any]] = None,
        clock: Callable[[], float] = time.monotonic,
        hour: Callable[[], int] = lambda: datetime.now().hour,
    ) -> None:
        self._say_sink = say
        self._precache = precache
        self._clock = clock
        self._hour = hour
        self._lock = threading.Lock()

        self._state: Optional[str] = None
        self._point_index: Optional[int] = None
        self._run_id: Optional[str] = None
        self._spoken_point_events: set[tuple[Any, ...]] = set()

        self._rtk_announced: str = "none"
        self._rtk_announced_at = -1.0e9

        self._battery_low_warned = False
        self._battery_critical_warned = False

        self._link_seen = False
        self._link_connected = False
        self._link_lost_announced = False

        self._obstacle_zone: Optional[str] = None
        self._obstacle_warned_at = -1.0e9
        self._obstacle_sensor_lost_announced = False

    # ------------------------------------------------------------ helpers

    def _say(self, key: str, **params: Any) -> None:
        self._say_sink(key, priority=key in PRIORITY_PHRASES, **params)

    def _heading(self, point_index: Optional[int]) -> None:
        if point_index is not None and point_index >= 0:
            self._say("heading_to_target", n=point_index + 1)

    # ------------------------------------------------------------ mission

    def on_mission_status(self, payload: dict[str, Any]) -> None:
        try:
            with self._lock:
                self._mission_status_locked(payload)
        except Exception:
            LOGGER.exception("Voice: mission status handling failed")

    def _mission_status_locked(self, payload: dict[str, Any]) -> None:
        state = _state(payload.get("state"))
        if not state:
            return
        raw_index = payload.get("current_point_index")
        point_index = raw_index if isinstance(raw_index, int) else None
        run_id = payload.get("mission_run_id")
        if run_id != self._run_id:
            self._spoken_point_events.clear()
            self._run_id = run_id

        previous, previous_index = self._state, self._point_index
        self._state, self._point_index = state, point_index

        if previous is None:
            return  # backend just started: learn the state, say nothing

        if state == previous:
            # AUTO: the next point is selected while still RUNNING.
            if state == "RUNNING" and point_index != previous_index:
                self._heading(point_index)
            return

        if state == "READY" and previous not in ACTIVE_STATES:
            self._say("mission_ready")
            self._schedule_precache(payload)
        elif state == "RUNNING":
            if previous == "PAUSED":
                self._say("mission_resumed")
            elif previous != "WAITING_FOR_NEXT":
                self._say("mission_started")
            self._heading(point_index)
        elif state == "PAUSED":
            reason = _state(payload.get("pause_reason"))
            self._say(PAUSE_REASON_PHRASES.get(reason, "mission_paused"))
        elif state == "COMPLETED":
            self._say("mission_completed")
        elif state == "ERROR":
            self._say("mission_error")
        elif previous in ACTIVE_STATES and state in {"EMPTY", "READY"}:
            self._say("mission_stopped")

    def _schedule_precache(self, payload: dict[str, Any]) -> None:
        if self._precache is None:
            return
        total = payload.get("total_points")
        if not isinstance(total, int) or total <= 0:
            return
        laser = _state(payload.get("marking_tool")) == "LASER"
        per_point = (
            ("heading_to_target", "target_laser_mark", "target_laser_do_not_mark")
            if laser
            else ("heading_to_target", "target_done")
        )
        items = [(key, {"n": n}) for n in range(1, total + 1) for key in per_point]
        self._precache(items)

    def on_point_event(self, payload: dict[str, Any]) -> None:
        try:
            with self._lock:
                self._point_event_locked(payload)
        except Exception:
            LOGGER.exception("Voice: point event handling failed")

    def _point_event_locked(self, payload: dict[str, Any]) -> None:
        event = _state(payload.get("event"))
        index = payload.get("point_index")
        if not isinstance(index, int) or index < 0:
            return
        key = (payload.get("mission_run_id"), payload.get("point_id"), event)
        if key in self._spoken_point_events:
            return
        spray = payload.get("spray") if isinstance(payload.get("spray"), dict) else {}
        laser = _state(spray.get("tool")) == "LASER"
        n = index + 1

        if event == "COMPLETED":
            self._say("target_laser_mark" if laser else "target_done", n=n)
        elif event == "ACCURACY_FAILED":
            reason = str(payload.get("reason") or "")
            if laser and reason.startswith(LASER_MEASURED_MISS_PREFIX):
                self._say("target_laser_do_not_mark", n=n)
            else:
                self._say("target_failed", n=n)
        elif event == "SKIPPED":
            self._say("target_skipped", n=n)
        else:
            return
        self._spoken_point_events.add(key)

    # ------------------------------------------------------------ RTK

    def on_gps_fix_type(self, fix_type: int) -> None:
        try:
            with self._lock:
                self._gps_locked(int(fix_type))
        except Exception:
            LOGGER.exception("Voice: GPS fix handling failed")

    def _gps_locked(self, fix_type: int) -> None:
        category = "fixed" if fix_type == 6 else "float" if fix_type == 5 else "gps_only"
        if category == self._rtk_announced:
            return
        now = self._clock()
        if now - self._rtk_announced_at < RTK_ANNOUNCE_COOLDOWN_SEC:
            return  # announced later if the new state persists
        if self._state in RTK_QUIET_STATES:
            return

        key: Optional[str] = None
        if category == "fixed":
            key = "rtk_fixed"
        elif category == "float" and self._rtk_announced == "fixed":
            key = "rtk_float"
        elif category == "gps_only" and self._rtk_announced in {"fixed", "float"}:
            key = "rtk_lost"

        self._rtk_announced = category
        if key is not None:
            self._rtk_announced_at = now
            self._say(key)

    # ------------------------------------------------------------ battery

    def on_battery_percent(self, percent: Optional[float]) -> None:
        try:
            with self._lock:
                if percent is None or percent < 0:
                    return
                pct = int(round(percent))
                if pct > 25:
                    self._battery_low_warned = False
                    self._battery_critical_warned = False
                elif pct <= 10 and not self._battery_critical_warned:
                    self._battery_critical_warned = True
                    self._battery_low_warned = True
                    self._say("battery_critical", pct=pct)
                elif pct <= 20 and not self._battery_low_warned:
                    self._battery_low_warned = True
                    self._say("battery_low", pct=pct)
        except Exception:
            LOGGER.exception("Voice: battery handling failed")

    # ------------------------------------------------------------ vehicle link

    def on_vehicle_connected(self, connected: bool) -> None:
        try:
            with self._lock:
                self._link_locked(bool(connected))
        except Exception:
            LOGGER.exception("Voice: link handling failed")

    def _link_locked(self, connected: bool) -> None:
        if not self._link_seen:
            self._link_seen = True
            self._link_connected = connected
            if connected:
                self._say("startup")
                self._say(phrases.greeting_key(self._hour()))
            return
        if connected == self._link_connected:
            return
        self._link_connected = connected
        if not connected:
            if self._state in ACTIVE_STATES:
                self._link_lost_announced = True
                self._say("connection_lost")
        elif self._link_lost_announced:
            self._link_lost_announced = False
            self._say("connection_restored")

    # ------------------------------------------------------------ obstacle

    def on_obstacle_status(self, payload: dict[str, Any]) -> None:
        try:
            with self._lock:
                self._obstacle_locked(payload)
        except Exception:
            LOGGER.exception("Voice: obstacle handling failed")

    def _obstacle_locked(self, payload: dict[str, Any]) -> None:
        if not payload.get("enabled"):
            self._obstacle_zone = None
            self._obstacle_sensor_lost_announced = False
            return
        healthy = bool(payload.get("healthy"))
        zone = str(payload.get("zone") or "unknown")
        previous, self._obstacle_zone = self._obstacle_zone, zone

        if not healthy:
            if self._state in ACTIVE_STATES and not self._obstacle_sensor_lost_announced:
                self._obstacle_sensor_lost_announced = True
                self._say("obstacle_sensor_lost")
            return
        self._obstacle_sensor_lost_announced = False

        if zone == "warning" and previous != "warning" and self._state == "RUNNING":
            now = self._clock()
            if now - self._obstacle_warned_at >= OBSTACLE_WARNING_COOLDOWN_SEC:
                distance = payload.get("distance_mm")
                if isinstance(distance, (int, float)) and distance > 0:
                    self._obstacle_warned_at = now
                    self._say("obstacle_ahead", cm=int(round(distance / 100.0)) * 10)

    # ------------------------------------------------------------ static cache

    def static_phrases(self) -> list[tuple[str, dict[str, Any]]]:
        """Fixed phrases worth rendering at startup and on language change."""
        return [
            (key, {})
            for key, variants in phrases.PHRASES.items()
            if "{" not in variants[phrases.DEFAULT_LANGUAGE]
        ]
