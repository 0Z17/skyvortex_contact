from pathlib import Path
import importlib.util


def _load_safety_module():
    module_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "safety"
        / "safety_monitor_node.py"
    )
    spec = importlib.util.spec_from_file_location("safety_monitor_node", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_roll_limit_triggers_state():
    module = _load_safety_module()
    state = module.evaluate_safety(roll_deg=20, pitch_deg=0, max_roll=12, max_pitch=12)
    assert state == "ATTITUDE_LIMIT_EXCEEDED"
