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


def test_map_pwm_to_magnitude_maps_min_to_max_range():
    module = _load_rc_manager_module()

    assert module.map_pwm_to_magnitude(1044, 1044, 1494, 1944, 50) == 0.0
    assert module.map_pwm_to_magnitude(1944, 1044, 1494, 1944, 50) == pytest.approx(1.0)
    assert module.map_pwm_to_magnitude(1494, 1044, 1494, 1944, 50) == pytest.approx(400.0 / 850.0)


def test_direction_and_magnitude_to_speed_combines_two_rc_topics():
    module_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "utils"
        / "rc_to_dynamixel_speed.py"
    )
    spec = importlib.util.spec_from_file_location("rc_to_dynamixel_speed", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert module.direction_and_magnitude_to_speed(1, 0.5, 20.0, 0.02) == pytest.approx(10.0)
    assert module.direction_and_magnitude_to_speed(-1, 0.5, 20.0, 0.02) == pytest.approx(-10.0)
    assert module.direction_and_magnitude_to_speed(0, 1.0, 20.0, 0.02) == 0.0
    assert module.direction_and_magnitude_to_speed(1, 0.01, 20.0, 0.02) == 0.0
