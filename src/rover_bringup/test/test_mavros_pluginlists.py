"""The rover loads only the MAVROS plugins it uses; keep the ones it needs."""

from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
PLUGINLISTS = PKG / "config" / "mavros_pluginlists.yaml"
LAUNCH = PKG / "launch" / "rover.launch.py"

# Plugin -> what breaks without it.
REQUIRED = {
    "sys_status": "/mavros/state, set_mode, battery, set_message_interval",
    "sys_time": "FCU time sync for message timestamps",
    "command": "arming, laser/spray actuator commands",
    "param": "FCU parameter capture",
    "setpoint_raw": "OFFBOARD velocity/yaw setpoints",
    "local_position": "/mavros/local_position/odom",
    "global_position": "raw/fix, gp_origin, global",
    "gps_rtk": "RTCM injection through PX4",
    "gps_status": "fix type for the RTK gates",
    "imu": "imu/data",
}


def _params():
    return yaml.safe_load(PLUGINLISTS.read_text(encoding="utf-8"))["/**"]["ros__parameters"]


def test_everything_else_is_denied():
    assert _params()["plugin_denylist"] == ["*"]


def test_required_plugins_are_allowed():
    allowed = set(_params()["plugin_allowlist"])
    missing = {name: why for name, why in REQUIRED.items() if name not in allowed}
    assert not missing, missing


def test_launch_uses_the_rover_list_with_stock_fallback():
    source = LAUNCH.read_text(encoding="utf-8")
    assert '"mavros_pluginlists.yaml"' in source
    assert "os.path.realpath(__file__)" in source
    assert "px4_pluginlists.yaml" in source  # fallback
    assert 'f"{MAVROS_PLUGINLISTS_YAML} "' in source
