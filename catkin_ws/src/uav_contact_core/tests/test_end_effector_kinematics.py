from pathlib import Path
import importlib.util


def _load_end_effector_module():
    module_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "end_effector"
        / "end_effector_kinematics_node.py"
    )
    spec = importlib.util.spec_from_file_location("end_effector_kinematics_node", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_compute_twist_returns_zero_without_trajectory():
    module = _load_end_effector_module()
    controller = module.EndEffectorTwistController()

    twist = controller.compute_twist()

    assert twist.linear.x == 0.0
    assert twist.linear.y == 0.0


def test_compute_twist_generates_xy_velocity_from_trajectory_velocity():
    module = _load_end_effector_module()
    controller = module.EndEffectorTwistController(link_length=0.7835, vel_factor=2.0, max_xy_speed=0.4)

    class _Traj:
        x = 0.01
        y = 0.0
        z = 1.0
        psi = 0.0
        theta = 0.0
        vx = 0.3
        vy = 0.1
        vz = 0.0
        vpsi = 0.0
        vtheta = 0.0
        nx = 1.0
        ny = 0.0
        nz = 0.0

    controller.update_trajectory_reference(_Traj())
    twist = controller.compute_twist()

    assert abs(twist.linear.x) <= 0.4
    assert abs(twist.linear.y) <= 0.4
