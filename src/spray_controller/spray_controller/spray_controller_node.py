#!/usr/bin/env python3
"""Production spray-servo controller for the DYX 4WD marking rover.

PX4 contract
------------
Holybro Pixhawk 6X AUX5:
    PWM_AUX_FUNC5 = 301
    Peripheral via Actuator Set 1

MAVLink command:
    MAV_CMD_DO_SET_ACTUATOR = 187
    param1 = actuator-set-1 normalized value [-1, +1]

Mission contract
----------------
The mission_manager publishes /marking_active only while a REAL marking point
is inside tolerance and the rover is stationary. This node independently
re-validates mission state, point identity, PX4 state, safety topics and topic
freshness before moving the spray servo.

Accuracy contract: with require_raw_gnss_spray_gate (production default) the
servo is pressed only when mission_manager's raw-GNSS spray gate reports PASS
for this exact run/point with a raw RTK-FIXED radial error <=
raw_gnss_max_radial_mm (30 mm). RPP's estimator-frame numbers never authorize
spray.

Safety properties
-----------------
* Non-blocking state machine (no sleep in callbacks/timers).
* Never reports SUCCESS until PRESS was accepted and RELEASE was accepted.
* PRESS is never automatically retried after an uncertain command timeout.
* RELEASE is idempotently retried until PX4 acknowledges it.
* Mission sprays release on E-stop, mission-disable, OFFBOARD loss, disarm,
  stale control input, or marking-hold loss. Manual sprays bypass only the
  mission-specific gates and still release on E-stop/PX4 safety loss.
* Manual PRESS is bounded below the independent hard press watchdog; repeated
  requests never extend the original deadline.
* A persistent journal prevents a node restart from blindly re-spraying a
  point after a crash at an uncertain stage.
* Faults latch. Reset is allowed only in a safe state using /spray/reset_fault.
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path
from typing import Any, Optional

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class SprayController(Node):
    """Fail-safe AUX5 spray-servo controller."""

    MAV_CMD_DO_SET_ACTUATOR = 187
    MAV_RESULT_ACCEPTED = 0
    PROTOCOL_VERSION = 1

    STATE_IDLE = "IDLE"
    STATE_WAIT_PRESS_ACK = "WAIT_PRESS_ACK"
    STATE_SPRAYING = "SPRAYING"
    STATE_WAIT_RELEASE_ACK = "WAIT_RELEASE_ACK"
    STATE_RECOVERY_RELEASE = "RECOVERY_RELEASE"
    STATE_WAIT_MARKING_CLEAR = "WAIT_MARKING_CLEAR"
    STATE_FAULT = "FAULT"

    TRANSACTION_MISSION = "MISSION"
    TRANSACTION_MANUAL = "MANUAL"

    # Marking tool, chosen per mission by mission_manager (status
    # "marking_tool"). SPRAY: timed PRESS/RELEASE on AUX5. LASER: ON at the
    # verified point and held until mission_manager clears "laser_hold"
    # (operator NEXT/SKIP/STOP), then OFF, on AUX6.
    TOOL_SPRAY = "SPRAY"
    TOOL_LASER = "LASER"
    STATE_LASER_ON = "LASER_ON"
    # Raw GNSS measured the stop OUTSIDE tolerance: blink the laser (rover is
    # here, do not mark) until mission_manager clears laser_hold.
    STATE_LASER_BLINK = "LASER_BLINK"
    LASER_STEADY = "STEADY"
    LASER_BLINK = "BLINK"
    ACTIVE_MISSION_STATES = frozenset({"RUNNING", "PAUSED", "WAITING_FOR_NEXT"})

    JOURNAL_PRESS_COMMAND_SENT = "PRESS_COMMAND_SENT"
    JOURNAL_PRESSED = "PRESSED"
    JOURNAL_RELEASE_UNCONFIRMED = "RELEASE_UNCONFIRMED"
    JOURNAL_COMPLETED = "COMPLETED"
    JOURNAL_FAILED_SAFE = "FAILED_SAFE"
    # Spray cut short by an operator/mission stop with the RELEASE
    # acknowledged by PX4. Not uncertain, so it neither latches a fault nor
    # triggers recovery on restart.
    JOURNAL_ABORTED_BY_STOP = "ABORTED_BY_STOP"

    # Abort reasons that an operator STOP / E-stop produces. The node sees
    # /marking_active, /mission_enable, /emergency_stop and the raw-GNSS gate
    # in /mission_manager/status drop together, in whichever order they
    # arrive, so any of the four can be recorded first.
    STOP_ABORT_REASONS = frozenset(
        {
            "MARKING_HOLD_LOST_DURING_SPRAY",
            "MISSION_DISABLED_DURING_SPRAY",
            "EMERGENCY_STOP_DURING_SPRAY",
            "RAW_GNSS_SPRAY_GATE_LOST_DURING_SPRAY",
        }
    )
    OPERATOR_CANCEL_REASONS = frozenset(
        {
            "MANUAL_RELEASE_REQUESTED",
            "MANUAL_OVERRIDE_TIMEOUT",
            "CONTROLLER_DISABLED_DURING_SPRAY",
        }
    )

    def __init__(self) -> None:
        super().__init__("spray_controller")

        # ==========================================================
        # Parameters
        # ==========================================================
        self.declare_parameter("enabled", True)
        self.declare_parameter("press_value", -0.25)
        self.declare_parameter("release_value", -1.0)
        self.declare_parameter("spray_duration_sec", 0.20)
        self.declare_parameter("pre_spray_stable_sec", 0.05)

        self.declare_parameter("command_timeout_sec", 1.0)
        self.declare_parameter("release_retry_interval_sec", 0.25)
        self.declare_parameter("hard_press_timeout_sec", 5.0)
        self.declare_parameter("manual_override_timeout_sec", 3.0)

        # MAV_CMD_DO_SET_ACTUATOR param index = PX4 "Peripheral via Actuator
        # Set N" (PWM_AUX_FUNCx = 300 + N). Spray AUX5 = set 1 (301), laser
        # AUX6 = set 2 (302).
        self.declare_parameter("spray_actuator_set", 1)
        self.declare_parameter("laser_actuator_set", 2)
        # PWM_AUX_MIN6/MAX6 = 1000/2000 -> 2000 us = +1.0 (ON), 1000 us =
        # -1.0 (OFF). PWM_AUX_DIS6 must equal the OFF PWM so PX4 turns the
        # laser off whenever it disarms.
        self.declare_parameter("laser_on_value", 1.0)
        self.declare_parameter("laser_off_value", -1.0)
        # After the laser point reports SUCCESS, mission_manager needs a
        # moment to publish laser_hold for it; the laser stays on through
        # this handoff only while that point is still the active one.
        self.declare_parameter("laser_handoff_timeout_sec", 1.5)
        # BLINK: ON for this long, OFF for this long, repeated until NEXT.
        self.declare_parameter("laser_blink_half_period_sec", 0.5)

        self.declare_parameter("mavros_state_timeout_sec", 2.5)
        self.declare_parameter("mission_status_timeout_sec", 1.0)
        self.declare_parameter("marking_active_timeout_sec", 0.50)

        self.declare_parameter("require_px4_armed", True)
        self.declare_parameter("require_px4_offboard", True)
        # Independent re-check of mission_manager's raw-GNSS spray gate.
        self.declare_parameter("require_raw_gnss_spray_gate", True)
        self.declare_parameter("raw_gnss_max_radial_mm", 30.0)

        self.declare_parameter(
            "journal_path",
            "~/.ros/dyx_spray_controller_journal.json",
        )
        # Operator-saved settings (POST /api/spray/duration). A valid saved
        # spray_duration_sec overrides the launch value on every start.
        self.declare_parameter(
            "spray_settings_path",
            "~/.ros/dyx_spray_settings.json",
        )

        self.enabled = bool(self.get_parameter("enabled").value)
        self.press_value = float(self.get_parameter("press_value").value)
        self.release_value = float(self.get_parameter("release_value").value)
        self.spray_duration_sec = float(self.get_parameter("spray_duration_sec").value)
        self.pre_spray_stable_sec = float(
            self.get_parameter("pre_spray_stable_sec").value
        )

        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )
        self.release_retry_interval_sec = float(
            self.get_parameter("release_retry_interval_sec").value
        )
        self.hard_press_timeout_sec = float(
            self.get_parameter("hard_press_timeout_sec").value
        )
        self.manual_override_timeout_sec = float(
            self.get_parameter("manual_override_timeout_sec").value
        )
        self.spray_actuator_set = int(
            self.get_parameter("spray_actuator_set").value
        )
        self.laser_actuator_set = int(
            self.get_parameter("laser_actuator_set").value
        )
        self.laser_on_value = float(self.get_parameter("laser_on_value").value)
        self.laser_off_value = float(self.get_parameter("laser_off_value").value)
        self.laser_handoff_timeout_sec = float(
            self.get_parameter("laser_handoff_timeout_sec").value
        )
        self.laser_blink_half_period_sec = float(
            self.get_parameter("laser_blink_half_period_sec").value
        )

        self.mavros_state_timeout_sec = float(
            self.get_parameter("mavros_state_timeout_sec").value
        )
        self.mission_status_timeout_sec = float(
            self.get_parameter("mission_status_timeout_sec").value
        )
        self.marking_active_timeout_sec = float(
            self.get_parameter("marking_active_timeout_sec").value
        )

        self.require_px4_armed = bool(self.get_parameter("require_px4_armed").value)
        self.require_px4_offboard = bool(
            self.get_parameter("require_px4_offboard").value
        )
        self.require_raw_gnss_spray_gate = bool(
            self.get_parameter("require_raw_gnss_spray_gate").value
        )
        self.raw_gnss_max_radial_mm = float(
            self.get_parameter("raw_gnss_max_radial_mm").value
        )

        self.journal_path = Path(
            os.path.expanduser(str(self.get_parameter("journal_path").value))
        )

        self._last_config_request_id: Optional[str] = None
        self._last_config_result: Optional[str] = None
        self._last_config_reason: Optional[str] = None

        self._validate_parameters()

        self.spray_settings_path = Path(
            os.path.expanduser(str(self.get_parameter("spray_settings_path").value))
        )
        self.spray_duration_source = "launch"
        self.laser_value_source = "launch"
        self._apply_saved_settings()

        # ==========================================================
        # QoS
        # ==========================================================
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        retained_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ==========================================================
        # Inputs
        # ==========================================================
        self.create_subscription(
            Bool,
            "/marking_active",
            self._marking_active_callback,
            command_qos,
        )

        self.create_subscription(
            Bool,
            "/mission_enable",
            self._mission_enable_callback,
            command_qos,
        )

        self.create_subscription(
            Bool,
            "/emergency_stop",
            self._emergency_stop_callback,
            command_qos,
        )

        self.create_subscription(
            String,
            "/mission_manager/status",
            self._mission_status_callback,
            retained_qos,
        )

        self.create_subscription(
            State,
            "/mavros/state",
            self._mavros_state_callback,
            state_qos,
        )

        # ==========================================================
        # Outputs
        # ==========================================================

        self.create_subscription(
            String,
            "/spray/config",
            self._spray_config_callback,
            retained_qos,
        )
        self.create_subscription(
            String,
            "/spray/laser_config",
            self._laser_config_callback,
            # Volatile, like /spray/duration_config: persisted in the file.
            command_qos,
        )
        self.create_subscription(
            String,
            "/spray/duration_config",
            self._spray_duration_config_callback,
            # Volatile on purpose: the settings file is the persistence. A
            # retained request that was rejected (controller busy) must not
            # be replayed and silently applied on a later restart.
            command_qos,
        )

        self.spray_active_pub = self.create_publisher(
            Bool,
            "/spray/active",
            retained_qos,
        )

        self.spray_complete_pub = self.create_publisher(
            String,
            "/spray/complete",
            retained_qos,
        )

        self.spray_result_pub = self.create_publisher(
            String,
            "/spray/result",
            retained_qos,
        )

        self.spray_status_pub = self.create_publisher(
            String,
            "/spray/status",
            retained_qos,
        )

        self.create_service(
            Trigger,
            "/spray/reset_fault",
            self._reset_fault_service,
        )
        self.create_service(
            SetBool,
            "/spray/manual",
            self._manual_spray_service,
        )
        self.create_service(
            SetBool,
            "/spray/set_enabled",
            self._set_enabled_service,
        )

        # ==========================================================
        # MAVROS command client
        # ==========================================================
        self.command_client = self.create_client(
            CommandLong,
            "/mavros/cmd/command",
        )

        # ==========================================================
        # Runtime inputs/state
        # ==========================================================
        self.marking_active = False
        self.mission_enable = False
        self.emergency_stop = True

        self.mavros_connected = False
        self.px4_armed = False
        self.px4_mode = ""

        self.mission_state: Optional[str] = None
        self.mission_run_id: Optional[str] = None
        self.current_point_id: Optional[str] = None
        self.current_point_index: Optional[int] = None
        self.current_point_state: Optional[str] = None
        self.status_marking_active = False
        self.raw_gnss_spray_gate: Optional[dict[str, Any]] = None
        self.mission_marking_tool = self.TOOL_SPRAY
        self.laser_hold_run_id: Optional[str] = None
        self.laser_hold_point_id: Optional[str] = None
        self.laser_hold_mode: Optional[str] = None

        self.last_marking_active_rx: Optional[float] = None
        self.last_mission_status_rx: Optional[float] = None
        self.last_mavros_state_rx: Optional[float] = None

        self.marking_active_since: Optional[float] = None

        # Monotonic transition timestamps of the latest transaction, for field
        # latency measurement only. Written once per transition, never read by
        # any gate. Same CLOCK_MONOTONIC as Mission Manager/RPP on this host.
        self.transaction_timing: dict[str, float] = {}

        # ==========================================================
        # Spray transaction state
        # ==========================================================
        self.state = self.STATE_IDLE

        self.transaction_run_id: Optional[str] = None
        self.transaction_point_id: Optional[str] = None
        self.transaction_point_index: Optional[int] = None
        self.transaction_kind: Optional[str] = None
        self.transaction_tool: Optional[str] = None
        # Set when a LASER transaction has reported SUCCESS (laser confirmed
        # ON); from then on, turning it off is the normal end, not a fault.
        self.laser_on_since: Optional[float] = None
        # BLINK transaction: the ON/OFF toggle in flight and the last one PX4
        # acknowledged. A toggle is only sent once the previous one is ACKed.
        self.laser_blink = False
        self.laser_blink_target: Optional[bool] = None
        self.laser_blink_is_on = False
        self.laser_blink_sent_at: Optional[float] = None
        self.laser_blink_next_toggle_at = 0.0

        self.manual_requested = False
        self.manual_deadline_at: Optional[float] = None

        self.command_future = None
        self.command_kind: Optional[str] = None
        self.command_sent_at: Optional[float] = None

        self.press_may_be_active = False
        self.press_command_sent_at: Optional[float] = None
        self.spray_started_at: Optional[float] = None

        self.failure_after_release: Optional[str] = None
        self.next_release_retry_at = 0.0
        self.preserve_fault_after_release = False

        self.release_confirmed = False

        self.fault_latched = False
        self.fault_reason: Optional[str] = None

        self.result_sequence = 0
        self.last_result: Optional[dict[str, Any]] = None
        self._last_completed_replay_key: Optional[tuple[str, str]] = None

        self.journal = self._load_journal()

        if self.journal is not None:
            journal_state = str(self.journal.get("state", ""))

            if journal_state in {
                self.JOURNAL_PRESS_COMMAND_SENT,
                self.JOURNAL_PRESSED,
                self.JOURNAL_RELEASE_UNCONFIRMED,
            }:
                self.transaction_run_id = self._journal_str("mission_run_id")
                self.transaction_point_id = self._journal_str("point_id")
                self.transaction_point_index = self._journal_int("point_index")
                journal_kind = self._journal_str("transaction_kind")
                self.transaction_kind = (
                    journal_kind
                    if journal_kind
                    in {self.TRANSACTION_MISSION, self.TRANSACTION_MANUAL}
                    else self.TRANSACTION_MISSION
                )
                journal_tool = self._journal_str("transaction_tool")
                self.transaction_tool = (
                    self.TOOL_LASER
                    if journal_tool == self.TOOL_LASER
                    else self.TOOL_SPRAY
                )
                self.press_may_be_active = True
                self.failure_after_release = "NODE_RESTART_DURING_UNCERTAIN_SPRAY"
                self.state = self.STATE_RECOVERY_RELEASE
                self.next_release_retry_at = 0.0

                self.get_logger().error(
                    "Uncertain previous spray transaction found in journal; "
                    "release recovery is required before operation."
                )

            elif journal_state == self.JOURNAL_FAILED_SAFE:
                self.fault_latched = True
                self.release_confirmed = True
                self.fault_reason = str(
                    self.journal.get(
                        "reason",
                        "PREVIOUS_SPRAY_FAILED",
                    )
                )
                self.state = self.STATE_FAULT

        self._publish_active(self.press_may_be_active)
        self._publish_status()

        # 50 Hz safety/control state machine.
        self.create_timer(0.02, self._control_loop)

        # 5 Hz retained health/status heartbeat.
        self.create_timer(0.20, self._publish_status)

        self.get_logger().warn("===== PRODUCTION SPRAY CONTROLLER STARTED =====")
        self.get_logger().warn(
            "PX4 output contract: AUX5 / PWM_AUX_FUNC5=301 / "
            "Peripheral via Actuator Set 1"
        )
        self.get_logger().warn(
            f"Servo command: press={self.press_value:+.3f}, "
            f"duration={self.spray_duration_sec:.3f}s, "
            f"release={self.release_value:+.3f}"
        )
        self.get_logger().warn(f"Journal: {self.journal_path}")
        self.get_logger().warn(
            f"Spray settings: {self.spray_settings_path} "
            f"(duration source={self.spray_duration_source})"
        )
        self.get_logger().warn(
            f"Laser: AUX via Actuator Set {self.laser_actuator_set} "
            f"on={self.laser_on_value:+.3f} off={self.laser_off_value:+.3f} "
            f"(source={self.laser_value_source})"
        )

    # ==============================================================
    # Parameter / utility helpers
    # ==============================================================

    def _validate_parameters(self) -> None:
        for name, value in (
            ("press_value", self.press_value),
            ("release_value", self.release_value),
            ("laser_on_value", self.laser_on_value),
            ("laser_off_value", self.laser_off_value),
        ):
            if not math.isfinite(value) or not -1.0 <= value <= 1.0:
                raise ValueError(f"{name} must be finite and in [-1.0, +1.0]")

        positive = {
            "spray_duration_sec": self.spray_duration_sec,
            "pre_spray_stable_sec": self.pre_spray_stable_sec,
            "command_timeout_sec": self.command_timeout_sec,
            "release_retry_interval_sec": self.release_retry_interval_sec,
            "hard_press_timeout_sec": self.hard_press_timeout_sec,
            "manual_override_timeout_sec": self.manual_override_timeout_sec,
            "mavros_state_timeout_sec": self.mavros_state_timeout_sec,
            "mission_status_timeout_sec": self.mission_status_timeout_sec,
            "marking_active_timeout_sec": self.marking_active_timeout_sec,
            "raw_gnss_max_radial_mm": self.raw_gnss_max_radial_mm,
            "laser_handoff_timeout_sec": self.laser_handoff_timeout_sec,
            "laser_blink_half_period_sec": self.laser_blink_half_period_sec,
        }

        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")

        if self.hard_press_timeout_sec <= self.spray_duration_sec:
            raise ValueError(
                "hard_press_timeout_sec must be greater than " "spray_duration_sec"
            )

        if self.manual_override_timeout_sec >= self.hard_press_timeout_sec:
            raise ValueError(
                "manual_override_timeout_sec must be strictly less than "
                "hard_press_timeout_sec"
            )

        for name, value in (
            ("spray_actuator_set", self.spray_actuator_set),
            ("laser_actuator_set", self.laser_actuator_set),
        ):
            if not 1 <= value <= 6:
                raise ValueError(f"{name} must be in 1..6")

        if self.spray_actuator_set == self.laser_actuator_set:
            raise ValueError("spray and laser must use different actuator sets")

        if self.laser_on_value == self.laser_off_value:
            raise ValueError("laser_on_value must differ from laser_off_value")

    @staticmethod
    def _age(last_rx: Optional[float], now: float) -> float:
        if last_rx is None:
            return math.inf
        return max(0.0, now - last_rx)

    def _journal_str(self, key: str) -> Optional[str]:
        if self.journal is None:
            return None
        value = self.journal.get(key)
        if isinstance(value, str) and value:
            return value
        return None

    def _journal_int(self, key: str) -> Optional[int]:
        if self.journal is None:
            return None
        value = self.journal.get(key)
        if isinstance(value, int):
            return value
        return None

    # ==============================================================
    # Persistent transaction journal
    # ==============================================================

    def _load_journal(self) -> Optional[dict[str, Any]]:
        try:
            if not self.journal_path.exists():
                return None

            payload = json.loads(self.journal_path.read_text(encoding="utf-8"))

            if not isinstance(payload, dict):
                raise ValueError("journal root is not an object")

            if int(payload.get("version", -1)) != self.PROTOCOL_VERSION:
                raise ValueError("unsupported journal version")

            return payload

        except Exception as exc:
            # A corrupt safety journal must not be silently ignored.
            self.get_logger().error(f"Unable to read spray journal safely: {exc}")
            return {
                "version": self.PROTOCOL_VERSION,
                "state": self.JOURNAL_FAILED_SAFE,
                "reason": f"JOURNAL_READ_ERROR: {exc}",
                "timestamp_unix_ns": time.time_ns(),
            }

    def _write_journal(
        self,
        *,
        state: str,
        reason: Optional[str] = None,
    ) -> None:
        payload = {
            "version": self.PROTOCOL_VERSION,
            "state": state,
            "mission_run_id": self.transaction_run_id,
            "point_id": self.transaction_point_id,
            "point_index": self.transaction_point_index,
            "transaction_kind": self.transaction_kind,
            "transaction_tool": self.transaction_tool,
            "press_value": self.press_value,
            "release_value": self.release_value,
            "spray_duration_sec": self.spray_duration_sec,
            "reason": reason,
            "timestamp_unix_ns": time.time_ns(),
        }

        self.journal_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        temporary = self.journal_path.with_suffix(self.journal_path.suffix + ".tmp")

        encoded = json.dumps(
            payload,
            separators=(",", ":"),
            sort_keys=True,
        )

        with temporary.open("w", encoding="utf-8") as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())

        os.replace(
            temporary,
            self.journal_path,
        )

        self.journal = payload

    def _clear_journal(self) -> None:
        try:
            self.journal_path.unlink(missing_ok=True)
        except Exception as exc:
            raise RuntimeError(f"Unable to clear spray journal: {exc}") from exc
        self.journal = None

    # ==============================================================
    # Input callbacks
    # ==============================================================

    def _marking_active_callback(self, msg: Bool) -> None:
        now = time.monotonic()
        new_value = bool(msg.data)

        self.last_marking_active_rx = now

        if new_value and not self.marking_active:
            self.marking_active_since = now

        if not new_value and self.marking_active:
            self.marking_active_since = None

            if self.transaction_kind != self.TRANSACTION_MANUAL and self.state in {
                self.STATE_WAIT_PRESS_ACK,
                self.STATE_SPRAYING,
            }:
                self._abort_to_release("MARKING_HOLD_LOST_DURING_SPRAY")

            elif (
                self.transaction_kind != self.TRANSACTION_MANUAL
                and self.state == self.STATE_WAIT_MARKING_CLEAR
            ):
                self._reset_transaction_to_idle()

        self.marking_active = new_value

    def _mission_enable_callback(self, msg: Bool) -> None:
        self.mission_enable = bool(msg.data)

        if (
            self.transaction_kind != self.TRANSACTION_MANUAL
            and not self.mission_enable
            and self.state in {
                self.STATE_WAIT_PRESS_ACK,
                self.STATE_SPRAYING,
            }
        ):
            self._abort_to_release("MISSION_DISABLED_DURING_SPRAY")

    def _emergency_stop_callback(self, msg: Bool) -> None:
        self.emergency_stop = bool(msg.data)

        if self.emergency_stop and self.state in {
            self.STATE_WAIT_PRESS_ACK,
            self.STATE_SPRAYING,
            self.STATE_LASER_ON,
            self.STATE_LASER_BLINK,
        }:
            self._abort_to_release("EMERGENCY_STOP_DURING_SPRAY")

    def _mission_status_callback(self, msg: String) -> None:
        now = time.monotonic()

        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        if not isinstance(payload, dict):
            return

        self.last_mission_status_rx = now

        state = payload.get("state")
        self.mission_state = str(state) if state is not None else None

        run_id = payload.get("mission_run_id")
        self.mission_run_id = (
            str(run_id) if isinstance(run_id, str) and run_id else None
        )

        point_id = payload.get("current_point_id")
        self.current_point_id = (
            str(point_id) if isinstance(point_id, str) and point_id else None
        )

        point_index = payload.get("current_point_index")
        self.current_point_index = (
            int(point_index) if isinstance(point_index, int) else None
        )

        point_state = payload.get("current_point_state")
        self.current_point_state = str(point_state) if point_state is not None else None

        self.status_marking_active = bool(payload.get("marking_active", False))

        tool = str(payload.get("marking_tool") or self.TOOL_SPRAY).strip().upper()
        self.mission_marking_tool = (
            self.TOOL_LASER if tool == self.TOOL_LASER else self.TOOL_SPRAY
        )
        hold = payload.get("laser_hold")
        if isinstance(hold, dict):
            hold_run = hold.get("mission_run_id")
            hold_point = hold.get("point_id")
            self.laser_hold_run_id = (
                str(hold_run) if isinstance(hold_run, str) and hold_run else None
            )
            self.laser_hold_point_id = (
                str(hold_point)
                if isinstance(hold_point, str) and hold_point
                else None
            )
            mode = str(hold.get("mode") or self.LASER_STEADY).strip().upper()
            self.laser_hold_mode = (
                self.LASER_BLINK if mode == self.LASER_BLINK else self.LASER_STEADY
            )
        else:
            self.laser_hold_run_id = None
            self.laser_hold_point_id = None
            self.laser_hold_mode = None

        raw_gate = payload.get("raw_gnss_spray_gate")
        self.raw_gnss_spray_gate = raw_gate if isinstance(raw_gate, dict) else None
        if (
            self.transaction_kind != self.TRANSACTION_MANUAL
            and self.require_raw_gnss_spray_gate
            and self.state in {self.STATE_WAIT_PRESS_ACK, self.STATE_SPRAYING}
            and not self._raw_gnss_gate_ok()
        ):
            self._abort_to_release("RAW_GNSS_SPRAY_GATE_LOST_DURING_SPRAY")

    def _mavros_state_callback(self, msg: State) -> None:
        self.last_mavros_state_rx = time.monotonic()

        self.mavros_connected = bool(msg.connected)
        self.px4_armed = bool(msg.armed)
        self.px4_mode = str(msg.mode or "")

        if self.state in {
            self.STATE_WAIT_PRESS_ACK,
            self.STATE_SPRAYING,
            self.STATE_LASER_ON,
            self.STATE_LASER_BLINK,
        } and not self._transaction_px4_gate_ok(time.monotonic()):
            self._abort_to_release("PX4_STATE_BECAME_UNSAFE_DURING_SPRAY")

    # ==============================================================
    # Safety gates
    # ==============================================================

    def _px4_gate_ok(self, now: float, *, require_offboard: bool = True) -> bool:
        if self._age(self.last_mavros_state_rx, now) > self.mavros_state_timeout_sec:
            return False

        if not self.mavros_connected:
            return False

        if self.require_px4_armed and not self.px4_armed:
            return False

        if (
            require_offboard
            and self.require_px4_offboard
            and self.px4_mode.upper() != "OFFBOARD"
        ):
            return False

        return True

    def _transaction_px4_gate_ok(self, now: float) -> bool:
        """Apply the PX4 gate appropriate to the active transaction kind."""
        if self.transaction_kind == self.TRANSACTION_MANUAL:
            return (
                self._age(self.last_mavros_state_rx, now)
                <= self.mavros_state_timeout_sec
                and self.mavros_connected
                and self.px4_armed
            )
        return self._px4_gate_ok(now)

    def _spray_config_callback(
        self,
        msg: String,
    ) -> None:
        """Safely update servo press/release positions at runtime."""

        try:
            payload = json.loads(msg.data)
        except Exception:
            self.get_logger().error("Rejected spray configuration: invalid JSON")
            return

        if not isinstance(payload, dict):
            self.get_logger().error(
                "Rejected spray configuration: payload is not an object"
            )
            return

        request_id = str(payload.get("request_id") or "").strip()

        if self._reject_config_if_busy(request_id):
            return

        try:
            press_value = float(payload["press_value"])

            release_value = float(payload["release_value"])

        except (
            KeyError,
            TypeError,
            ValueError,
        ):
            self._last_config_request_id = request_id
            self._last_config_result = "REJECTED"
            self._last_config_reason = "INVALID_VALUE"

            self._publish_status()
            return

        if not math.isfinite(press_value) or not -1.0 <= press_value <= 1.0:
            self._last_config_request_id = request_id
            self._last_config_result = "REJECTED"
            self._last_config_reason = "PRESS_VALUE_OUT_OF_RANGE"

            self._publish_status()
            return

        if not math.isfinite(release_value) or not -1.0 <= release_value <= 1.0:
            self._last_config_request_id = request_id
            self._last_config_result = "REJECTED"
            self._last_config_reason = "RELEASE_VALUE_OUT_OF_RANGE"

            self._publish_status()
            return

        self.press_value = press_value
        self.release_value = release_value

        self._last_config_request_id = request_id
        self._last_config_result = "ACCEPTED"
        self._last_config_reason = None

        self.get_logger().warn(
            "Spray servo configuration updated: "
            f"press={self.press_value:+.3f}, "
            f"release={self.release_value:+.3f}"
        )

        self._publish_status()

    def _reject_config_if_busy(self, request_id: str) -> bool:
        """Reject a configuration change unless the controller is idle."""

        # Configuration must NEVER change during a spray transaction.
        if (
            self.state != self.STATE_IDLE
            or self.press_may_be_active
            or self.marking_active
            or self.mission_enable
        ):
            self.get_logger().warn(
                "Rejected spray configuration because " "spray controller is not idle"
            )

            self._last_config_request_id = request_id
            self._last_config_result = "REJECTED"
            self._last_config_reason = "SPRAY_CONTROLLER_NOT_IDLE"

            self._publish_status()
            return True

        return False

    def _spray_duration_in_range(self, value: float) -> bool:
        return (
            math.isfinite(value)
            and 0.0 < value <= self.manual_override_timeout_sec
            and value < self.hard_press_timeout_sec
        )

    def _laser_value_in_range(self, value: float) -> bool:
        return math.isfinite(value) and -1.0 <= value <= 1.0

    def _read_saved_settings(self) -> dict[str, Any]:
        if not self.spray_settings_path.exists():
            return {}
        payload = json.loads(self.spray_settings_path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("settings file is not a JSON object")
        return payload

    def _apply_saved_settings(self) -> None:
        """Apply operator-saved values over the launch values.

        Each saved key is applied on its own. A missing file keeps the launch
        values. A corrupt file, or an out-of-range value, is logged and
        ignored rather than stopping the controller.
        """

        try:
            payload = self._read_saved_settings()
        except Exception as exc:
            self.get_logger().error(
                f"Ignoring unreadable spray settings {self.spray_settings_path}: "
                f"{exc}; using launch values"
            )
            return

        if "spray_duration_sec" in payload:
            try:
                value = float(payload["spray_duration_sec"])
            except (TypeError, ValueError):
                value = math.nan
            if self._spray_duration_in_range(value):
                self.spray_duration_sec = value
                self.spray_duration_source = "saved"
            else:
                self.get_logger().error(
                    f"Ignoring saved spray_duration_sec="
                    f"{payload['spray_duration_sec']!r} (out of range); using "
                    f"launch value {self.spray_duration_sec:.3f}"
                )

        if "laser_on_value" in payload or "laser_off_value" in payload:
            try:
                on_value = float(payload["laser_on_value"])
                off_value = float(payload["laser_off_value"])
            except (KeyError, TypeError, ValueError):
                on_value = off_value = math.nan
            if (
                self._laser_value_in_range(on_value)
                and self._laser_value_in_range(off_value)
                and on_value != off_value
            ):
                self.laser_on_value = on_value
                self.laser_off_value = off_value
                self.laser_value_source = "saved"
            else:
                self.get_logger().error(
                    "Ignoring saved laser values (invalid); using launch "
                    f"on={self.laser_on_value:+.3f} off={self.laser_off_value:+.3f}"
                )

    def _write_saved_settings(self, **updates: float) -> None:
        """Merge updates into the settings file, atomically.

        Only keys the operator has explicitly saved are stored, so a later
        change to a launch value still applies to every other setting.
        """
        try:
            payload = self._read_saved_settings()
        except Exception:
            payload = {}

        payload.update(updates)
        payload["version"] = 1
        payload["updated_unix_ns"] = time.time_ns()

        self.spray_settings_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        temporary = self.spray_settings_path.with_suffix(
            self.spray_settings_path.suffix + ".tmp"
        )

        with temporary.open("w", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, separators=(",", ":"), sort_keys=True))
            handle.flush()
            os.fsync(handle.fileno())

        os.replace(
            temporary,
            self.spray_settings_path,
        )

    def _laser_config_callback(
        self,
        msg: String,
    ) -> None:
        """Safely update and save the laser ON/OFF actuator values."""

        try:
            payload = json.loads(msg.data)
        except Exception:
            self.get_logger().error("Rejected laser configuration: invalid JSON")
            return

        if not isinstance(payload, dict):
            self.get_logger().error(
                "Rejected laser configuration: payload is not an object"
            )
            return

        request_id = str(payload.get("request_id") or "").strip()

        if self._reject_config_if_busy(request_id):
            return

        try:
            on_value = float(payload["laser_on_value"])
            off_value = float(payload["laser_off_value"])
        except (
            KeyError,
            TypeError,
            ValueError,
        ):
            self._last_config_request_id = request_id
            self._last_config_result = "REJECTED"
            self._last_config_reason = "INVALID_VALUE"

            self._publish_status()
            return

        if (
            not self._laser_value_in_range(on_value)
            or not self._laser_value_in_range(off_value)
            or on_value == off_value
        ):
            self._last_config_request_id = request_id
            self._last_config_result = "REJECTED"
            self._last_config_reason = "LASER_VALUE_OUT_OF_RANGE"

            self._publish_status()
            return

        try:
            self._write_saved_settings(
                laser_on_value=on_value,
                laser_off_value=off_value,
            )
        except Exception as exc:
            self.get_logger().error(f"Unable to save laser values: {exc}")

            self._last_config_request_id = request_id
            self._last_config_result = "REJECTED"
            self._last_config_reason = "SPRAY_SETTINGS_SAVE_FAILED"

            self._publish_status()
            return

        self.laser_on_value = on_value
        self.laser_off_value = off_value
        self.laser_value_source = "saved"

        self._last_config_request_id = request_id
        self._last_config_result = "ACCEPTED"
        self._last_config_reason = None

        self.get_logger().warn(
            "Laser configuration updated: "
            f"on={self.laser_on_value:+.3f}, off={self.laser_off_value:+.3f}"
        )

        self._publish_status()

    def _spray_duration_config_callback(
        self,
        msg: String,
    ) -> None:
        """Safely update the mission spray hold time at runtime.

        The upper bound is the manual override timeout, so every accepted
        duration can also be reproduced with the manual spray test, and it
        stays strictly below the independent hard press watchdog.
        """

        try:
            payload = json.loads(msg.data)
        except Exception:
            self.get_logger().error(
                "Rejected spray duration configuration: invalid JSON"
            )
            return

        if not isinstance(payload, dict):
            self.get_logger().error(
                "Rejected spray duration configuration: payload is not an object"
            )
            return

        request_id = str(payload.get("request_id") or "").strip()

        if self._reject_config_if_busy(request_id):
            return

        try:
            spray_duration_sec = float(payload["spray_duration_sec"])
        except (
            KeyError,
            TypeError,
            ValueError,
        ):
            self._last_config_request_id = request_id
            self._last_config_result = "REJECTED"
            self._last_config_reason = "INVALID_VALUE"

            self._publish_status()
            return

        if not self._spray_duration_in_range(spray_duration_sec):
            self._last_config_request_id = request_id
            self._last_config_result = "REJECTED"
            self._last_config_reason = "SPRAY_DURATION_OUT_OF_RANGE"

            self._publish_status()
            return

        # Save first: an accepted value must survive a restart. If the file
        # cannot be written, reject instead of applying a value that would
        # silently revert to the launch default on the next start.
        try:
            self._write_saved_settings(spray_duration_sec=spray_duration_sec)
        except Exception as exc:
            self.get_logger().error(f"Unable to save spray hold time: {exc}")

            self._last_config_request_id = request_id
            self._last_config_result = "REJECTED"
            self._last_config_reason = "SPRAY_SETTINGS_SAVE_FAILED"

            self._publish_status()
            return

        self.spray_duration_sec = spray_duration_sec
        self.spray_duration_source = "saved"

        self._last_config_request_id = request_id
        self._last_config_result = "ACCEPTED"
        self._last_config_reason = None

        self.get_logger().warn(
            "Spray hold time updated: "
            f"spray_duration_sec={self.spray_duration_sec:.3f}"
        )

        self._publish_status()

    def _spray_gate_ok(self, now: float) -> bool:
        if not self.enabled:
            return False

        if self.fault_latched:
            return False

        if not self.mission_enable or self.emergency_stop:
            return False

        if self.mission_state != "RUNNING":
            return False

        if not self.mission_run_id:
            return False

        if not self.current_point_id:
            return False

        if self.current_point_state != "ACTIVE":
            return False

        if not self.marking_active or not self.status_marking_active:
            return False

        if (
            self._age(self.last_marking_active_rx, now)
            > self.marking_active_timeout_sec
        ):
            return False

        if (
            self._age(self.last_mission_status_rx, now)
            > self.mission_status_timeout_sec
        ):
            return False

        if not self._px4_gate_ok(now):
            return False

        if not self._raw_gnss_gate_ok():
            return False

        if not self.command_client.service_is_ready():
            return False

        return True

    def _manual_gate_failure(self, now: float) -> Optional[str]:
        """Return why a new manual PRESS cannot be accepted, if anything."""
        if not self.enabled:
            return "SPRAY_CONTROLLER_DISABLED"
        if self.fault_latched:
            return "SPRAY_FAULT_LATCHED"
        if self.state != self.STATE_IDLE or self.press_may_be_active:
            return "SPRAY_CONTROLLER_NOT_IDLE"
        if self.emergency_stop:
            return "EMERGENCY_STOP_ACTIVE"
        if (
            self._age(self.last_mission_status_rx, now)
            > self.mission_status_timeout_sec
        ):
            return "MISSION_STATUS_STALE"
        if self.mission_state in self.ACTIVE_MISSION_STATES:
            return f"MISSION_ACTIVE:{self.mission_state}"
        if self.mission_state is None:
            return "MISSION_STATE_UNKNOWN"
        if not self._transaction_px4_gate_ok_for_manual(now):
            return "PX4_STATE_UNSAFE"
        if not self.command_client.service_is_ready():
            return "MAVROS_COMMAND_SERVICE_UNAVAILABLE"
        return None

    def _transaction_px4_gate_ok_for_manual(self, now: float) -> bool:
        """Manual spray always requires fresh, connected, armed PX4 state."""
        return (
            self._age(self.last_mavros_state_rx, now)
            <= self.mavros_state_timeout_sec
            and self.mavros_connected
            and self.px4_armed
        )

    def _active_transaction_gate_ok(self, now: float) -> bool:
        """Re-check the safety contract for the active spray transaction."""
        if self.transaction_kind != self.TRANSACTION_MANUAL:
            return self._spray_gate_ok(now)

        return (
            self.enabled
            and not self.fault_latched
            and not self.emergency_stop
            and self._age(self.last_mission_status_rx, now)
            <= self.mission_status_timeout_sec
            and self.mission_state not in self.ACTIVE_MISSION_STATES
            and self.mission_state is not None
            and self._transaction_px4_gate_ok_for_manual(now)
            and self.command_client.service_is_ready()
        )

    def _laser_hold_failure(self, now: float) -> Optional[str]:
        """Why a laser held ON must now be switched OFF, or None to keep it.

        OFF is the normal end once mission_manager clears laser_hold for
        this point (NEXT, SKIP, STOP, completion). It is also forced by any
        safety loss. There is deliberately no time limit on a valid hold.
        """
        if not self.enabled:
            return "LASER_CONTROLLER_DISABLED"
        if self.emergency_stop:
            return "EMERGENCY_STOP_DURING_SPRAY"
        if self.mission_marking_tool != self.TOOL_LASER:
            return "LASER_TOOL_DESELECTED"
        if self._age(self.last_mission_status_rx, now) > self.mission_status_timeout_sec:
            return "MISSION_STATUS_STALE_DURING_LASER"
        if not self._px4_gate_ok(now):
            return "PX4_STATE_BECAME_UNSAFE_DURING_SPRAY"

        expected_mode = self.LASER_BLINK if self.laser_blink else self.LASER_STEADY
        if (
            self.laser_hold_run_id == self.transaction_run_id
            and self.laser_hold_point_id == self.transaction_point_id
            and self.transaction_point_id is not None
            and self.laser_hold_mode == expected_mode
        ):
            return None

        # Handoff: the SUCCESS result is out but mission_manager has not yet
        # published the hold. Tolerate that only while this point is still
        # the running mission's active point. A blink starts FROM the hold,
        # so it has no handoff.
        if (
            not self.laser_blink
            and self.laser_on_since is not None
            and now - self.laser_on_since < self.laser_handoff_timeout_sec
            and self.mission_state == "RUNNING"
            and self._transaction_matches_current()
        ):
            return None

        return "LASER_HOLD_RELEASED"

    def _laser_blink_requested(self, now: float) -> bool:
        """mission_manager asks for a BLINK hold and it is safe to start one."""
        return (
            self.mission_marking_tool == self.TOOL_LASER
            and self.laser_hold_mode == self.LASER_BLINK
            and self.laser_hold_point_id is not None
            and self.laser_hold_run_id is not None
            and self.laser_hold_run_id == self.mission_run_id
            and self.enabled
            and not self.fault_latched
            and not self.emergency_stop
            and not self.press_may_be_active
            and self._age(self.last_mission_status_rx, now)
            <= self.mission_status_timeout_sec
            and self._px4_gate_ok(now)
            and self.command_client.service_is_ready()
        )

    def _begin_laser_blink(self, now: float) -> None:
        self.transaction_kind = self.TRANSACTION_MISSION
        self.transaction_tool = self.TOOL_LASER
        self.transaction_run_id = self.laser_hold_run_id
        self.transaction_point_id = self.laser_hold_point_id
        self.transaction_point_index = None
        self.transaction_timing = {"laser_blink_start": now}
        self.failure_after_release = None
        self.laser_blink = True
        # From the first ON, turning the laser off is the normal end.
        self.laser_on_since = now

        # Persist before the first ON, so a restart turns the laser OFF.
        self._write_journal(state=self.JOURNAL_PRESS_COMMAND_SENT)
        self.press_may_be_active = True
        self.release_confirmed = False
        self._publish_active(True)
        self.state = self.STATE_LASER_BLINK
        self._send_laser_blink_toggle(True, now)
        self.get_logger().warn(
            f"LASER BLINK | {self.transaction_point_id} | outside tolerance, "
            "waiting for NEXT"
        )

    def _send_laser_blink_toggle(self, turn_on: bool, now: float) -> None:
        if not self._send_command(
            kind="PRESS" if turn_on else "RELEASE",
            value=self.laser_on_value if turn_on else self.laser_off_value,
        ):
            self._abort_to_release("LASER_BLINK_COMMAND_SERVICE_UNAVAILABLE")
            return
        self.laser_blink_target = turn_on
        self.laser_blink_sent_at = now

    def _laser_blink_step(self, now: float) -> None:
        # 1. Resolve the toggle in flight; never send over an outstanding one.
        if self.command_future is not None:
            if not self.command_future.done():
                if (
                    self.command_sent_at is not None
                    and now - self.command_sent_at > self.command_timeout_sec
                ):
                    self._clear_command_future()
                    self._abort_to_release("LASER_BLINK_ACK_TIMEOUT")
                return
            try:
                accepted = self._response_accepted(self.command_future.result())
            except Exception:
                accepted = False
            self._clear_command_future()
            if not accepted:
                self._abort_to_release("LASER_BLINK_COMMAND_REJECTED")
                return
            self.laser_blink_is_on = bool(self.laser_blink_target)
            self.laser_blink_next_toggle_at = (
                (self.laser_blink_sent_at or now) + self.laser_blink_half_period_sec
            )

        # 2. A cancel (OFF/disable) that arrived while a toggle was in flight.
        if self.failure_after_release is not None:
            self._begin_or_retry_release()
            return

        # 3. Hold released or safety lost -> OFF.
        reason = self._laser_hold_failure(now)
        if reason is not None:
            self._abort_to_release(reason)
            return

        # 4. Next half-period.
        if now >= self.laser_blink_next_toggle_at:
            self._send_laser_blink_toggle(not self.laser_blink_is_on, now)

    def _raw_gnss_gate_ok(self) -> bool:
        """Raw RTK-FIXED GNSS radial error <= raw_gnss_max_radial_mm, re-checked.

        Bound to the current run and point so a PASS for another point can
        never press the servo, and re-compared against this node's own limit
        so a loosened mission_manager tolerance cannot widen it.
        """
        if not self.require_raw_gnss_spray_gate:
            return True

        gate = self.raw_gnss_spray_gate
        if not isinstance(gate, dict):
            return False

        if gate.get("enabled") is not True:
            return False

        if gate.get("pass") is not True or gate.get("status") != "PASS":
            return False

        if gate.get("measurement_source") != "RAW_GNSS_SURVEY":
            return False

        if not self.mission_run_id or gate.get("mission_run_id") != self.mission_run_id:
            return False

        if not self.current_point_id or gate.get("point_id") != self.current_point_id:
            return False

        for key in ("radial_error_mm", "tolerance_mm"):
            value = gate.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return False
            if not math.isfinite(value) or value < 0.0:
                return False
            if value > self.raw_gnss_max_radial_mm:
                return False

        return True

    # ==============================================================
    # MAVLink command helpers
    # ==============================================================

    def _make_actuator_request(
        self,
        value: float,
        actuator_set: Optional[int] = None,
        extra: Optional[dict[int, float]] = None,
    ) -> CommandLong.Request:
        req = CommandLong.Request()
        req.broadcast = False
        req.command = self.MAV_CMD_DO_SET_ACTUATOR
        req.confirmation = 0

        # Actuator Set N -> paramN. NaN explicitly leaves the other sets
        # alone, so a laser command never moves the spray servo and back.
        values = {
            index: math.nan
            for index in range(1, 7)
        }
        values[actuator_set or self.spray_actuator_set] = float(value)
        for index, extra_value in (extra or {}).items():
            values[index] = float(extra_value)
        req.param1 = values[1]
        req.param2 = values[2]
        req.param3 = values[3]
        req.param4 = values[4]
        req.param5 = values[5]
        req.param6 = values[6]

        # Index 0 selects actuators 1..6. This is the first set.
        req.param7 = 0.0
        return req

    def _transaction_is_laser(self) -> bool:
        return self.transaction_tool == self.TOOL_LASER

    def _transaction_actuator_set(self) -> int:
        return (
            self.laser_actuator_set
            if self._transaction_is_laser()
            else self.spray_actuator_set
        )

    def _transaction_press_value(self) -> float:
        return self.laser_on_value if self._transaction_is_laser() else self.press_value

    def _transaction_release_value(self) -> float:
        return (
            self.laser_off_value
            if self._transaction_is_laser()
            else self.release_value
        )

    def _send_command(
        self,
        *,
        kind: str,
        value: float,
    ) -> bool:
        if not self.command_client.service_is_ready():
            return False

        self.command_future = self.command_client.call_async(
            self._make_actuator_request(value, self._transaction_actuator_set())
        )

        self.command_kind = kind
        self.command_sent_at = time.monotonic()
        return True

    @staticmethod
    def _response_accepted(response: Any) -> bool:
        if response is None:
            return False

        return (
            bool(getattr(response, "success", False))
            and int(getattr(response, "result", -1))
            == SprayController.MAV_RESULT_ACCEPTED
        )

    def _clear_command_future(self) -> None:
        self.command_future = None
        self.command_kind = None
        self.command_sent_at = None

    # ==============================================================
    # Transaction/result helpers
    # ==============================================================

    def _begin_transaction_from_current_point(self) -> None:
        self.transaction_kind = self.TRANSACTION_MISSION
        self.transaction_tool = self.mission_marking_tool
        self.transaction_run_id = self.mission_run_id
        self.transaction_point_id = self.current_point_id
        self.transaction_point_index = self.current_point_index

    def _transaction_matches_current(self) -> bool:
        return (
            self.transaction_run_id is not None
            and self.transaction_point_id is not None
            and self.transaction_run_id == self.mission_run_id
            and self.transaction_point_id == self.current_point_id
        )

    def _journal_completed_for_current_point(self) -> bool:
        if self.journal is None:
            return False

        return (
            self.journal.get("state") == self.JOURNAL_COMPLETED
            and self.journal.get("mission_run_id") == self.mission_run_id
            and self.journal.get("point_id") == self.current_point_id
        )

    def _publish_result(
        self,
        *,
        result: str,
        reason: Optional[str],
    ) -> None:
        self.result_sequence += 1

        payload = {
            "protocol_version": self.PROTOCOL_VERSION,
            "sequence": self.result_sequence,
            "result": result,
            "reason": reason,
            "mission_run_id": self.transaction_run_id,
            "point_id": self.transaction_point_id,
            "point_index": self.transaction_point_index,
            "press_value": self.press_value,
            "release_value": self.release_value,
            "spray_duration_sec": self.spray_duration_sec,
            "timestamp_unix_ns": time.time_ns(),
        }

        msg = String()
        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
            sort_keys=True,
        )
        self.spray_result_pub.publish(msg)
        self.last_result = payload

        if (
            result == "SUCCESS"
            and self.transaction_kind == self.TRANSACTION_MISSION
            and self.transaction_point_id is not None
        ):
            complete = String()
            complete.data = self.transaction_point_id
            self.spray_complete_pub.publish(complete)

    def _publish_active(self, active: bool) -> None:
        msg = Bool()
        msg.data = bool(active)
        self.spray_active_pub.publish(msg)

    def _controller_ready(self, now: float) -> bool:
        # Preflight readiness deliberately does NOT require armed/OFFBOARD,
        # because those gates may only be opened after the mission is started.
        return (
            self.enabled
            and not self.fault_latched
            and self.state
            not in {
                self.STATE_RECOVERY_RELEASE,
                self.STATE_FAULT,
            }
            and self.command_client.service_is_ready()
            and self.mavros_connected
            and self._age(
                self.last_mavros_state_rx,
                now,
            )
            <= self.mavros_state_timeout_sec
        )

    def _publish_status(self) -> None:
        now = time.monotonic()

        spray_elapsed_sec: Optional[float] = None
        spray_remaining_sec: Optional[float] = None
        if (
            self.state == self.STATE_SPRAYING
            and self.spray_started_at is not None
        ):
            spray_elapsed_sec = max(0.0, now - self.spray_started_at)
            spray_remaining_sec = max(
                0.0,
                self.spray_duration_sec - spray_elapsed_sec,
            )

        payload = {
            "protocol_version": self.PROTOCOL_VERSION,
            "timestamp_unix_ns": time.time_ns(),
            "ready": self._controller_ready(now),
            "enabled": self.enabled,
            "controller_state": self.state,
            "fault_latched": self.fault_latched,
            "fault_reason": self.fault_reason,
            "press_may_be_active": self.press_may_be_active,
            "release_confirmed": self.release_confirmed,
            "transaction_kind": self.transaction_kind,
            "manual_active": (
                self.transaction_kind == self.TRANSACTION_MANUAL
                and self.press_may_be_active
            ),
            "manual_requested": self.manual_requested,
            "manual_deadline_remaining_sec": (
                round(max(0.0, self.manual_deadline_at - now), 3)
                if self.manual_requested and self.manual_deadline_at is not None
                else None
            ),
            "mission_run_id": self.mission_run_id,
            "current_point_id": self.current_point_id,
            "transaction_run_id": self.transaction_run_id,
            "transaction_point_id": self.transaction_point_id,
            "mavros_connected": self.mavros_connected,
            "px4_armed": self.px4_armed,
            "px4_mode": self.px4_mode,
            "mission_state": self.mission_state,
            "mission_enable": self.mission_enable,
            "emergency_stop": self.emergency_stop,
            "marking_active": self.marking_active,
            "status_marking_active": self.status_marking_active,
            "require_raw_gnss_spray_gate": self.require_raw_gnss_spray_gate,
            "raw_gnss_max_radial_mm": self.raw_gnss_max_radial_mm,
            "raw_gnss_spray_gate_ok": self._raw_gnss_gate_ok(),
            "mavros_state_age_sec": (
                None
                if self.last_mavros_state_rx is None
                else round(
                    self._age(self.last_mavros_state_rx, now),
                    3,
                )
            ),
            "mission_status_age_sec": (
                None
                if self.last_mission_status_rx is None
                else round(
                    self._age(self.last_mission_status_rx, now),
                    3,
                )
            ),
            "marking_active_age_sec": (
                None
                if self.last_marking_active_rx is None
                else round(
                    self._age(self.last_marking_active_rx, now),
                    3,
                )
            ),
            "journal_state": (
                self.journal.get("state") if self.journal is not None else None
            ),
            "transaction_timing_monotonic": {
                name: round(value, 6)
                for name, value in self.transaction_timing.items()
                if value is not None
            },
            "press_value": self.press_value,
            "release_value": self.release_value,
            "spray_duration_sec": self.spray_duration_sec,
            "manual_override_timeout_sec": self.manual_override_timeout_sec,
            "spray_duration_source": self.spray_duration_source,
            "marking_tool": self.mission_marking_tool,
            "transaction_tool": self.transaction_tool,
            "laser_on": (
                self.transaction_tool == self.TOOL_LASER
                and self.press_may_be_active
            ),
            "laser_on_point_id": (
                self.transaction_point_id
                if self.transaction_tool == self.TOOL_LASER
                and self.press_may_be_active
                else None
            ),
            "laser_hold_point_id": self.laser_hold_point_id,
            "laser_hold_mode": self.laser_hold_mode,
            "laser_blinking": self.laser_blink and self.press_may_be_active,
            "laser_on_value": self.laser_on_value,
            "laser_off_value": self.laser_off_value,
            "laser_value_source": self.laser_value_source,
            "spray_actuator_set": self.spray_actuator_set,
            "laser_actuator_set": self.laser_actuator_set,
            "spray_elapsed_sec": (
                round(spray_elapsed_sec, 3)
                if spray_elapsed_sec is not None
                else None
            ),
            "spray_remaining_sec": (
                round(spray_remaining_sec, 3)
                if spray_remaining_sec is not None
                else None
            ),
            "spraying": self.state == self.STATE_SPRAYING,
            "config_request_id": (self._last_config_request_id),
            "config_result": (self._last_config_result),
            "config_reason": (self._last_config_reason),
            "last_result": self.last_result,
        }

        msg = String()
        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
            sort_keys=True,
        )
        self.spray_status_pub.publish(msg)

    # ==============================================================
    # Fault/release recovery
    # ==============================================================

    def _abort_to_release(self, reason: str) -> None:
        if self.failure_after_release is None:
            self.failure_after_release = reason

        self._write_journal(
            state=self.JOURNAL_RELEASE_UNCONFIRMED,
            reason=reason,
        )

        self._begin_or_retry_release()

    def _begin_or_retry_release(self) -> None:
        now = time.monotonic()

        # A RELEASE command may be repeated safely. Never create a second
        # outstanding future at the same time.
        if self.command_future is not None:
            return

        if self._send_command(
            kind="RELEASE",
            value=self._transaction_release_value(),
        ):
            # First RELEASE only; retries do not move the timestamp.
            self.transaction_timing.setdefault("release_sent", now)
            self.state = self.STATE_WAIT_RELEASE_ACK
            return

        self.state = self.STATE_RECOVERY_RELEASE
        self.next_release_retry_at = now + self.release_retry_interval_sec

    def _release_succeeded(self) -> None:
        self.transaction_timing["release_ack"] = time.monotonic()
        self.press_may_be_active = False
        self.release_confirmed = True
        self._publish_active(False)

        if self.preserve_fault_after_release:
            # /spray/manual=false may issue an extra idempotent RELEASE while
            # a non-manual fault is latched. Physical confirmation must not
            # reclassify that transaction or erase its FAILED_SAFE journal.
            self.preserve_fault_after_release = False
            self.manual_requested = False
            self.manual_deadline_at = None
            self.state = self.STATE_FAULT
            self.get_logger().warn(
                "Servo RELEASE confirmed; existing spray fault remains latched"
            )
            return

        if self._transaction_is_laser() and (
            self.laser_on_since is not None
            or self.failure_after_release == "NODE_RESTART_DURING_UNCERTAIN_SPRAY"
        ):
            # The laser point already reported SUCCESS (or a restart found
            # the laser possibly on). Laser OFF confirmed is the normal end:
            # never a fault, never a second result.
            point_id = self.transaction_point_id
            try:
                if self.laser_on_since is not None and not self.laser_blink:
                    self._write_journal(state=self.JOURNAL_COMPLETED)
                else:
                    self._clear_journal()
            except Exception as exc:
                self.get_logger().error(
                    f"Unable to update journal after LASER OFF: {exc}"
                )
            self._reset_transaction_to_idle()
            self.get_logger().warn(f"LASER OFF confirmed | {point_id}")
            return

        if self.transaction_kind == self.TRANSACTION_MANUAL:
            reason = self.failure_after_release
            if reason is not None and reason not in self.OPERATOR_CANCEL_REASONS:
                self.fault_latched = True
                self.fault_reason = reason
                self.manual_requested = False
                self.manual_deadline_at = None
                self.state = self.STATE_FAULT

                # RELEASE is physically confirmed, so a broken journal
                # filesystem must not throw out of the control loop and put
                # the process into a respawn cycle. Keep the fault latched in
                # memory and persist it on a best-effort basis.
                try:
                    self._write_journal(
                        state=self.JOURNAL_FAILED_SAFE,
                        reason=reason,
                    )
                except Exception as journal_exc:
                    self.get_logger().error(
                        "Unable to persist manual recovery fault after RELEASE: "
                        f"{journal_exc}"
                    )

                self.get_logger().error(
                    "Manual spray failed safely after RELEASE: " f"{reason}"
                )
                return

            # Manual transactions never publish a point result/completion.
            # Once RELEASE is acknowledged, no persistent replay record is
            # needed; a crash before this clear only causes another RELEASE.
            try:
                self._clear_journal()
            except Exception as exc:
                reason = f"MANUAL_JOURNAL_CLEANUP_FAILED:{exc}"
                self.fault_latched = True
                self.fault_reason = reason
                self.manual_requested = False
                self.manual_deadline_at = None
                self.state = self.STATE_FAULT

                # Best effort only: a second filesystem failure must not
                # escape the control loop after RELEASE has been confirmed.
                try:
                    self._write_journal(
                        state=self.JOURNAL_FAILED_SAFE,
                        reason=reason,
                    )
                except Exception as journal_exc:
                    self.get_logger().error(
                        "Unable to persist manual cleanup fault after RELEASE: "
                        f"{journal_exc}"
                    )

                self.get_logger().error(reason)
                return

            self._reset_transaction_to_idle()
            self.get_logger().warn("Manual spray RELEASE confirmed")
            return

        if self.failure_after_release is not None:
            reason = self.failure_after_release

            if self._aborted_by_stop(reason):
                # The mission was stopped mid-spray and PX4 confirmed the
                # servo is released: report the point as not sprayed, but do
                # not latch a fault that would silently block every later
                # mission until someone calls /spray/reset_fault.
                self._write_journal(
                    state=self.JOURNAL_ABORTED_BY_STOP,
                    reason=reason,
                )
                self._publish_result(
                    result="FAILED",
                    reason=reason,
                )
                self.state = self.STATE_WAIT_MARKING_CLEAR
                self.get_logger().warn(
                    "Spray cut short by mission stop; RELEASE confirmed, "
                    f"no fault latched: {reason}"
                )
                return

            self._write_journal(
                state=self.JOURNAL_FAILED_SAFE,
                reason=reason,
            )

            self._publish_result(
                result="FAILED",
                reason=reason,
            )

            self.fault_latched = True
            self.fault_reason = reason
            self.state = self.STATE_FAULT

            self.get_logger().error(
                f"Spray transaction failed safely after RELEASE: {reason}"
            )
            return

        self._write_journal(
            state=self.JOURNAL_COMPLETED,
        )

        self.transaction_timing["success"] = time.monotonic()
        self._publish_result(
            result="SUCCESS",
            reason=None,
        )

        self.state = self.STATE_WAIT_MARKING_CLEAR

        self.get_logger().warn(
            "SPRAY SUCCESS | "
            f"{self.transaction_point_id} | "
            f"press={self.press_value:+.3f} | "
            f"duration={self.spray_duration_sec:.3f}s | "
            f"release={self.release_value:+.3f} | "
            f"timing={self._transaction_timing_summary()}"
        )

    def _aborted_by_stop(self, reason: str) -> bool:
        """True when an abort was caused by the mission being stopped.

        Evaluated at RELEASE ACK time, after the stop's E-stop and
        mission-disable messages have had the PX4 round trip to arrive. A
        marking-hold loss while the mission is still enabled and the E-stop
        clear stays a latched fault.
        """
        if reason in self.OPERATOR_CANCEL_REASONS:
            return True
        return reason in self.STOP_ABORT_REASONS and (
            self.emergency_stop or not self.mission_enable
        )

    def _transaction_timing_summary(self) -> str:
        """Stage-to-stage latencies (ms) for the latest transaction."""
        order = (
            "marking_active_seen",
            "pre_spray_gate_valid",
            "press_sent",
            "press_ack",
            "release_sent",
            "release_ack",
            "success",
        )
        stamps = [
            (name, self.transaction_timing[name])
            for name in order
            if self.transaction_timing.get(name) is not None
        ]
        return " ".join(
            f"{a}->{b}={1000.0 * (tb - ta):.0f}ms"
            for (a, ta), (b, tb) in zip(stamps, stamps[1:])
        )

    def _set_fault_without_press(
        self,
        reason: str,
    ) -> None:
        self.fault_latched = True
        self.fault_reason = reason
        self.state = self.STATE_FAULT

        if self.transaction_kind == self.TRANSACTION_MANUAL:
            self._write_journal(
                state=self.JOURNAL_FAILED_SAFE,
                reason=reason,
            )
        elif (
            self.transaction_run_id is not None
            and self.transaction_point_id is not None
        ):
            self._write_journal(
                state=self.JOURNAL_FAILED_SAFE,
                reason=reason,
            )

            self._publish_result(
                result="FAILED",
                reason=reason,
            )

        self._publish_active(False)

        self.get_logger().error(f"SPRAY FAULT: {reason}")

    def _reset_transaction_to_idle(self) -> None:
        self.state = self.STATE_IDLE

        self.transaction_run_id = None
        self.transaction_point_id = None
        self.transaction_point_index = None
        self.transaction_kind = None
        self.transaction_tool = None
        self.laser_on_since = None
        self.laser_blink = False
        self.laser_blink_target = None
        self.laser_blink_is_on = False
        self.laser_blink_sent_at = None
        self.laser_blink_next_toggle_at = 0.0

        self.manual_requested = False
        self.manual_deadline_at = None

        self.command_future = None
        self.command_kind = None
        self.command_sent_at = None

        self.press_command_sent_at = None
        self.spray_started_at = None
        self.press_may_be_active = False

        self.failure_after_release = None
        self.next_release_retry_at = 0.0
        self.preserve_fault_after_release = False

        self.marking_active_since = None

        self._publish_active(False)

    # ==============================================================
    # Manual/runtime-control services
    # ==============================================================

    def _manual_spray_service(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        """Accept a bounded manual PRESS or an idempotent RELEASE request."""
        if not bool(request.data):
            self.manual_requested = False
            self.manual_deadline_at = None

            if self.fault_latched or self.state == self.STATE_FAULT:
                # Never turn a mission/manual FAILED_SAFE record into a fresh
                # manual transaction. An extra RELEASE is harmless, but its
                # ACK must return to FAULT without changing the journal.
                self.preserve_fault_after_release = True
                self._begin_or_retry_release()
                response.success = True
                response.message = (
                    "RELEASE accepted; existing spray fault remains latched"
                )
            elif not self.press_may_be_active and self.release_confirmed:
                response.success = True
                response.message = "Servo RELEASE already confirmed"
            elif self.state in {self.STATE_WAIT_PRESS_ACK, self.STATE_SPRAYING}:
                self._abort_to_release("MANUAL_RELEASE_REQUESTED")
                response.success = True
                response.message = "RELEASE accepted; confirmation pending"
            elif self.state in {
                self.STATE_WAIT_RELEASE_ACK,
                self.STATE_RECOVERY_RELEASE,
            }:
                response.success = True
                response.message = "RELEASE already pending"
            elif self.press_may_be_active:
                self.failure_after_release = "MANUAL_RELEASE_REQUESTED"
                self._write_journal(
                    state=self.JOURNAL_RELEASE_UNCONFIRMED,
                    reason=self.failure_after_release,
                )
                self._begin_or_retry_release()
                response.success = True
                response.message = "RELEASE accepted; confirmation pending"
            elif (
                self.state == self.STATE_IDLE
                and self.transaction_kind is None
                and self.journal is None
            ):
                # Even from IDLE, send an idempotent RELEASE so status can
                # distinguish an accepted request from physical confirmation.
                self.transaction_kind = self.TRANSACTION_MANUAL
                self.transaction_tool = self.TOOL_SPRAY
                self.failure_after_release = None
                self.release_confirmed = False
                self.transaction_timing = {}
                self._write_journal(state=self.JOURNAL_RELEASE_UNCONFIRMED)
                self._begin_or_retry_release()
                response.success = True
                response.message = "RELEASE accepted; confirmation pending"
            else:
                # Preserve completed/failed mission journal semantics and any
                # non-manual transaction that does not need another RELEASE.
                response.success = True
                response.message = "No active manual press; transaction preserved"

            self._publish_status()
            return response

        if (
            self.transaction_kind == self.TRANSACTION_MANUAL
            and self.manual_requested
            and self.state in {self.STATE_WAIT_PRESS_ACK, self.STATE_SPRAYING}
        ):
            response.success = True
            response.message = "Manual PRESS already active; deadline unchanged"
            return response

        now = time.monotonic()
        gate_failure = self._manual_gate_failure(now)
        if gate_failure is not None:
            response.success = False
            response.message = gate_failure
            return response

        self.transaction_kind = self.TRANSACTION_MANUAL
        self.transaction_tool = self.TOOL_SPRAY
        self.transaction_run_id = None
        self.transaction_point_id = None
        self.transaction_point_index = None
        self.transaction_timing = {"manual_request_accepted": now}
        self.manual_requested = True
        self.manual_deadline_at = None
        self.failure_after_release = None

        # Persist before PRESS transmission, exactly as mission spray does.
        self._write_journal(state=self.JOURNAL_PRESS_COMMAND_SENT)
        if not self._send_command(kind="PRESS", value=self.press_value):
            self.manual_requested = False
            self._set_fault_without_press(
                "MAVROS_COMMAND_SERVICE_UNAVAILABLE_BEFORE_MANUAL_PRESS"
            )
            response.success = False
            response.message = "MAVROS_COMMAND_SERVICE_UNAVAILABLE"
            return response

        self.press_command_sent_at = self.command_sent_at
        self.transaction_timing["press_sent"] = self.command_sent_at
        self.manual_deadline_at = (
            self.press_command_sent_at + self.manual_override_timeout_sec
        )
        self.press_may_be_active = True
        self.release_confirmed = False
        self._publish_active(True)
        self.state = self.STATE_WAIT_PRESS_ACK
        self._publish_status()

        response.success = True
        response.message = "Manual PRESS accepted; physical confirmation pending"
        return response

    def _set_enabled_service(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        """Set the runtime master gate; disabling releases any possible press."""
        requested = bool(request.data)
        self.enabled = requested

        if not requested:
            self.manual_requested = False
            self.manual_deadline_at = None
            if self.fault_latched or self.state == self.STATE_FAULT:
                # Disabling is allowed to issue an idempotent RELEASE, but a
                # pre-existing FAILED_SAFE transaction must remain intact
                # through ACK timeout/rejection/retry and final confirmation.
                self.preserve_fault_after_release = True
                self._begin_or_retry_release()
            elif self.state in {self.STATE_WAIT_PRESS_ACK, self.STATE_SPRAYING}:
                self._abort_to_release("CONTROLLER_DISABLED_DURING_SPRAY")
            elif self.press_may_be_active and self.state not in {
                self.STATE_WAIT_RELEASE_ACK,
                self.STATE_RECOVERY_RELEASE,
            }:
                self.failure_after_release = "CONTROLLER_DISABLED_DURING_SPRAY"
                self._write_journal(
                    state=self.JOURNAL_RELEASE_UNCONFIRMED,
                    reason=self.failure_after_release,
                )
                self._begin_or_retry_release()

        self._publish_status()
        response.success = True
        response.message = (
            "Spray controller enabled"
            if requested
            else "Spray controller disabled; RELEASE initiated if required"
        )
        return response

    # ==============================================================
    # Fault reset service
    # ==============================================================

    def _reset_fault_service(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if not self.fault_latched:
            response.success = True
            response.message = "Spray controller has no latched fault"
            return response

        if self.press_may_be_active:
            response.success = False
            response.message = "Cannot reset: servo release is not confirmed"
            return response

        if self.marking_active:
            response.success = False
            response.message = "Cannot reset while marking_active is true"
            return response

        if self.mission_enable and not self.emergency_stop:
            response.success = False
            response.message = "Cannot reset while mission is enabled without E-stop"
            return response

        try:
            self._clear_journal()
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response

        self.fault_latched = False
        self.fault_reason = None
        self.release_confirmed = True

        self._reset_transaction_to_idle()
        self._publish_status()

        response.success = True
        response.message = "Spray fault reset in safe state"
        return response

    # ==============================================================
    # Main control state machine
    # ==============================================================

    def _control_loop(self) -> None:
        now = time.monotonic()

        # ----------------------------------------------------------
        # Recovery after a node/process restart at an uncertain stage.
        # RELEASE is retried indefinitely until acknowledged.
        # ----------------------------------------------------------
        if self.state == self.STATE_RECOVERY_RELEASE:
            if self.command_future is None and now >= self.next_release_retry_at:
                self._begin_or_retry_release()
            return

        if self.state == self.STATE_FAULT:
            return

        # ----------------------------------------------------------
        # IDLE: wait for a stable REAL marking hold.
        # ----------------------------------------------------------
        if self.state == self.STATE_IDLE:
            if self._laser_blink_requested(now):
                self._begin_laser_blink(now)
                return

            if not self._spray_gate_ok(now):
                self.marking_active_since = now if self.marking_active else None
                return

            if self._journal_completed_for_current_point():
                key = (
                    str(self.mission_run_id),
                    str(self.current_point_id),
                )

                self._begin_transaction_from_current_point()

                if self._last_completed_replay_key != key:
                    self._publish_result(
                        result="SUCCESS",
                        reason="REPLAY_FROM_PERSISTENT_JOURNAL",
                    )
                    self._last_completed_replay_key = key

                self.state = self.STATE_WAIT_MARKING_CLEAR
                return

            if self.marking_active_since is None:
                self.marking_active_since = now
                return

            if now - self.marking_active_since < self.pre_spray_stable_sec:
                return

            self.transaction_timing = {
                "marking_active_seen": self.marking_active_since,
                "pre_spray_gate_valid": now,
            }
            self._begin_transaction_from_current_point()

            if not self._transaction_matches_current():
                self._set_fault_without_press("POINT_ID_CHANGED_BEFORE_PRESS")
                return

            # Persist the transaction BEFORE sending PRESS. If the process
            # dies in the tiny window around COMMAND_LONG transmission, the
            # restart path treats the press as uncertain, RELEASES, and faults
            # instead of blindly spraying the point a second time.
            self._write_journal(
                state=self.JOURNAL_PRESS_COMMAND_SENT,
            )

            # PRESS is intentionally sent at most once. If its ACK becomes
            # uncertain we RELEASE and fail instead of blindly pressing again.
            if not self._send_command(
                kind="PRESS",
                value=self._transaction_press_value(),
            ):
                self._set_fault_without_press(
                    "MAVROS_COMMAND_SERVICE_UNAVAILABLE_BEFORE_PRESS"
                )
                return

            self.press_command_sent_at = self.command_sent_at
            self.transaction_timing["press_sent"] = self.command_sent_at
            self.press_may_be_active = True
            self.release_confirmed = False
            self._publish_active(True)

            self.state = self.STATE_WAIT_PRESS_ACK
            return

        # ----------------------------------------------------------
        # PRESS ACK
        # ----------------------------------------------------------
        if self.state == self.STATE_WAIT_PRESS_ACK:
            if not self._active_transaction_gate_ok(now):
                self._abort_to_release("SAFETY_GATE_LOST_BEFORE_PRESS_ACK")
                return

            if (
                self.transaction_kind == self.TRANSACTION_MANUAL
                and self.manual_deadline_at is not None
                and now >= self.manual_deadline_at
            ):
                self.manual_requested = False
                self.manual_deadline_at = None
                self._abort_to_release("MANUAL_OVERRIDE_TIMEOUT")
                return

            if (
                self.press_command_sent_at is not None
                and now - self.press_command_sent_at >= self.hard_press_timeout_sec
            ):
                self._abort_to_release("HARD_PRESS_WATCHDOG_BEFORE_PRESS_ACK")
                return

            if self.command_future is None:
                self._abort_to_release("PRESS_FUTURE_MISSING")
                return

            if not self.command_future.done():
                if (
                    self.command_sent_at is not None
                    and now - self.command_sent_at > self.command_timeout_sec
                ):
                    self._clear_command_future()
                    self._abort_to_release("PRESS_ACK_TIMEOUT_UNCERTAIN")
                return

            try:
                response = self.command_future.result()
            except Exception as exc:
                self._clear_command_future()
                self._abort_to_release(f"PRESS_SERVICE_EXCEPTION:{exc}")
                return

            accepted = self._response_accepted(response)
            result_code = getattr(response, "result", None)
            self._clear_command_future()

            if not accepted:
                self._abort_to_release(f"PRESS_REJECTED_RESULT_{result_code}")
                return

            self._write_journal(
                state=self.JOURNAL_PRESSED,
            )
            self.transaction_timing["press_ack"] = now

            # The configured spray duration is the physical PRESS hold time.
            # Start the timer only after PX4/MAVROS has ACKed the PRESS command,
            # so command/transport latency is not subtracted from the configured
            # physical spray ON interval.
            self.spray_started_at = now

            # A cancellation/safety callback can arrive while the PRESS RPC
            # is in flight. Once that RPC resolves, RELEASE immediately
            # instead of entering a timed hold.
            if self.failure_after_release is not None:
                self._begin_or_retry_release()
                return

            if self._transaction_is_laser():
                # Laser confirmed ON: the point is marked. Report it now so
                # mission_manager completes the point and holds for NEXT; the
                # laser then stays on in LASER_ON until the hold is cleared.
                self.laser_on_since = now
                self.state = self.STATE_LASER_ON
                self._publish_result(result="SUCCESS", reason=None)
                self.get_logger().warn(
                    f"LASER ON | {self.transaction_point_id} | "
                    f"value={self.laser_on_value:+.3f} | waiting for NEXT"
                )
                return

            self.state = self.STATE_SPRAYING
            return

        # ----------------------------------------------------------
        # Timed press.
        # ----------------------------------------------------------
        if self.state == self.STATE_SPRAYING:
            if not self._active_transaction_gate_ok(now):
                self._abort_to_release("SAFETY_GATE_LOST_DURING_PRESS")
                return

            if self.press_command_sent_at is None:
                self._abort_to_release("PRESS_START_TIME_MISSING")
                return

            if self.spray_started_at is None:
                self._abort_to_release("SPRAY_HOLD_START_TIME_MISSING")
                return

            total_press_elapsed = now - self.press_command_sent_at
            spray_hold_elapsed = now - self.spray_started_at

            # Absolute fail-safe watchdog includes command/ACK latency.
            if total_press_elapsed >= self.hard_press_timeout_sec:
                self._abort_to_release("HARD_PRESS_WATCHDOG_EXPIRED")
                return

            # Normal release is based on the confirmed physical spray-hold
            # interval, not on the PRESS command transmission time.
            if (
                self.transaction_kind == self.TRANSACTION_MANUAL
                and self.manual_deadline_at is not None
                and now >= self.manual_deadline_at
            ):
                self.manual_requested = False
                self.manual_deadline_at = None
                self.failure_after_release = None
                self._begin_or_retry_release()
                return

            if (
                self.transaction_kind == self.TRANSACTION_MISSION
                and spray_hold_elapsed >= self.spray_duration_sec
            ):
                self.failure_after_release = None
                self._begin_or_retry_release()
            return

        # ----------------------------------------------------------
        # Laser held ON at a completed point until the operator moves on.
        # ----------------------------------------------------------
        if self.state == self.STATE_LASER_ON:
            reason = self._laser_hold_failure(now)
            if reason is not None:
                self._abort_to_release(reason)
            return

        if self.state == self.STATE_LASER_BLINK:
            self._laser_blink_step(now)
            return

        # ----------------------------------------------------------
        # RELEASE ACK. RELEASE may be retried; this is safe/idempotent.
        # ----------------------------------------------------------
        if self.state == self.STATE_WAIT_RELEASE_ACK:
            if self.command_future is None:
                self.state = self.STATE_RECOVERY_RELEASE
                self.next_release_retry_at = now
                return

            if not self.command_future.done():
                if (
                    self.command_sent_at is not None
                    and now - self.command_sent_at > self.command_timeout_sec
                ):
                    self._clear_command_future()

                    if not self.preserve_fault_after_release:
                        self._write_journal(
                            state=self.JOURNAL_RELEASE_UNCONFIRMED,
                            reason=(
                                self.failure_after_release or "RELEASE_ACK_TIMEOUT"
                            ),
                        )

                    self.state = self.STATE_RECOVERY_RELEASE
                    self.next_release_retry_at = now + self.release_retry_interval_sec
                return

            try:
                response = self.command_future.result()
            except Exception as exc:
                self._clear_command_future()

                if not self.preserve_fault_after_release:
                    self._write_journal(
                        state=self.JOURNAL_RELEASE_UNCONFIRMED,
                        reason=(
                            self.failure_after_release
                            or f"RELEASE_SERVICE_EXCEPTION:{exc}"
                        ),
                    )

                self.state = self.STATE_RECOVERY_RELEASE
                self.next_release_retry_at = now + self.release_retry_interval_sec
                return

            accepted = self._response_accepted(response)
            result_code = getattr(response, "result", None)
            self._clear_command_future()

            if not accepted:
                if not self.preserve_fault_after_release:
                    self._write_journal(
                        state=self.JOURNAL_RELEASE_UNCONFIRMED,
                        reason=(
                            self.failure_after_release
                            or f"RELEASE_REJECTED_RESULT_{result_code}"
                        ),
                    )

                self.state = self.STATE_RECOVERY_RELEASE
                self.next_release_retry_at = now + self.release_retry_interval_sec
                return

            self._release_succeeded()
            return

        # ----------------------------------------------------------
        # Do not accept another point until current marking hold clears.
        # ----------------------------------------------------------
        if self.state == self.STATE_WAIT_MARKING_CLEAR:
            if not self.marking_active:
                self._reset_transaction_to_idle()

    # ==============================================================
    # Shutdown
    # ==============================================================

    def best_effort_release_on_shutdown(self) -> None:
        """Try once to place the servo in release position before shutdown."""
        try:
            if not self.command_client.service_is_ready():
                return

            # One command: spray RELEASE and laser OFF together.
            future = self.command_client.call_async(
                self._make_actuator_request(
                    self.release_value,
                    self.spray_actuator_set,
                    extra={self.laser_actuator_set: self.laser_off_value},
                )
            )

            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=0.5,
            )

        except Exception:
            # Hardware/PX4 PWM_FAIL/PWM_DIS must remain the final protection
            # against process death or transport loss.
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SprayController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.best_effort_release_on_shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
