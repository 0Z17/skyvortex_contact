from pathlib import Path
import importlib.util

import pytest


def _load_rc_manager_module():
    module_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "utils"
        / "rc_manager_node.py"
    )
    spec = importlib.util.spec_from_file_location("rc_manager_node", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_map_pwm_to_axis_applies_deadband_and_scaling():
    module = _load_rc_manager_module()

    assert module.map_pwm_to_axis(1494, 1044, 1494, 1944, 50) == 0.0
    assert module.map_pwm_to_axis(1520, 1044, 1494, 1944, 50) == 0.0
    assert module.map_pwm_to_axis(1944, 1044, 1494, 1944, 50) == pytest.approx(1.0)
    assert module.map_pwm_to_axis(1044, 1044, 1494, 1944, 50) == pytest.approx(-1.0)


def test_map_pwm_to_axis_supports_invert():
    module = _load_rc_manager_module()

    assert module.map_pwm_to_axis(1944, 1044, 1494, 1944, 50, invert=True) == pytest.approx(-1.0)


def test_map_pwm_to_three_position_matches_legacy_switch_direction():
    module = _load_rc_manager_module()

    assert module.map_pwm_to_three_position(1494, 1044, 1494, 1944, 50) == 0
    assert module.map_pwm_to_three_position(1044, 1044, 1494, 1944, 50) == 1
    assert module.map_pwm_to_three_position(1944, 1044, 1494, 1944, 50) == -1


def test_map_pwm_to_three_position_rejects_out_of_range():
    module = _load_rc_manager_module()

    assert module.map_pwm_to_three_position(900, 1044, 1494, 1944, 50) == 0
    assert module.map_pwm_to_three_position(2100, 1044, 1494, 1944, 50) == 0
