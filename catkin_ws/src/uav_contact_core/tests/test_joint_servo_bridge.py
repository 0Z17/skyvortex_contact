from pathlib import Path
import importlib.util


def _load_joint_servo_bridge_module():
    module_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "actuation"
        / "joint_servo_bridge.py"
    )
    spec = importlib.util.spec_from_file_location("joint_servo_bridge", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_map_joint_angle_to_pwm_applies_limits_and_linear_mapping():
    module = _load_joint_servo_bridge_module()

    joint_min = -1.0
    joint_max = 1.0
    pwm_min = 1000
    pwm_max = 2000

    assert module.map_joint_angle_to_pwm(-2.0, joint_min, joint_max, pwm_min, pwm_max) == 1000
    assert module.map_joint_angle_to_pwm(2.0, joint_min, joint_max, pwm_min, pwm_max) == 2000
    assert module.map_joint_angle_to_pwm(0.0, joint_min, joint_max, pwm_min, pwm_max) == 1500
