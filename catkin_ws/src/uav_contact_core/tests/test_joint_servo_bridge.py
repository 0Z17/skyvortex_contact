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


def test_joint_servo_bridge_tracks_latest_theta_with_phase_gate():
    module = _load_joint_servo_bridge_module()
    bridge = module.JointServoBridge(joint_min=-1.0, joint_max=1.0, neutral_theta=0.2)

    bridge.update_theta(-0.5)
    assert bridge.current_joint_command() == 0.2

    bridge.set_phase_enabled(True)
    assert bridge.current_joint_command() == -0.5



def test_clamp_joint_angle_applies_limits():
    module = _load_joint_servo_bridge_module()

    assert module.clamp_joint_angle(-2.0, -1.0, 1.0) == -1.0
    assert module.clamp_joint_angle(2.0, -1.0, 1.0) == 1.0
    assert module.clamp_joint_angle(0.0, -1.0, 1.0) == 0.0
