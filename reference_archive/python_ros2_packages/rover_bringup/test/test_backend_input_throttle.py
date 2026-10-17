"""rover_backend's high-rate inputs go through a 10 Hz C++ relay."""

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / "rover_bringup" / "launch" / "rover.launch.py"
BRIDGE = ROOT / "rover_backend" / "rover_backend" / "ros_bridge.py"


def _assigned(path: Path, name: str):
    for node in ast.parse(path.read_text(encoding="utf-8")).body:
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == name for t in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError(f"{name} not found in {path.name}")


def test_launch_and_backend_agree_on_relay_topics():
    launch = dict(_assigned(LAUNCH, "BACKEND_THROTTLED_INPUTS"))
    backend = _assigned(BRIDGE, "THROTTLED_INPUTS")
    assert launch == backend


def test_relay_only_covers_display_inputs():
    # Mission, safety and RTK inputs must stay direct.
    names = set(dict(_assigned(LAUNCH, "BACKEND_THROTTLED_INPUTS")))
    assert names == {
        "/mavros/local_position/odom",
        "/mavros/global_position/global",
        "/mavros/global_position/compass_hdg",
        "/rpp/debug",
        "/rpp/accuracy",
    }
    assert _assigned(LAUNCH, "BACKEND_INPUT_THROTTLE_HZ") == 10.0


def test_backend_falls_back_to_direct_topics_without_the_relay():
    source = LAUNCH.read_text(encoding="utf-8")
    assert '"DYX_BACKEND_INPUT_THROTTLE": "1" if input_throttle else "0"' in source
    bridge = BRIDGE.read_text(encoding="utf-8")
    assert 'os.environ.get("DYX_BACKEND_INPUT_THROTTLE") == "1"' in bridge
