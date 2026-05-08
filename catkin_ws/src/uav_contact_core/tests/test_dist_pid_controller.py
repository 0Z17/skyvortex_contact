from pathlib import Path
import importlib.util
import sys
import types

import pytest


def _load_dist_pid_module():
    class _Float64:
        def __init__(self, data=0.0):
            self.data = float(data)

    class _Int8:
        def __init__(self, data=0):
            self.data = int(data)

    class _TaskPhase:
        INITIAL_CONTACT = 3
        SLIDING_CONTACT = 4

    class _ContactCommand:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)
            self.normal_direction = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.enabled = False
            self.normal_velocity = 0.0
            self.normal_offset = 0.0
            self.distance_error = 0.0
            self.measured_distance = 0.0
            self.desired_distance = 0.0

    sys.modules["rospy"] = types.SimpleNamespace()
    sys.modules["std_msgs.msg"] = types.SimpleNamespace(Float64=_Float64, Int8=_Int8)
    sys.modules["uav_contact_msgs.msg"] = types.SimpleNamespace(
        TaskPhase=_TaskPhase,
        ContactCommand=_ContactCommand,
    )

    module_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "contact"
        / "dist_pid_controller.py"
    )
    spec = importlib.util.spec_from_file_location("dist_pid_controller", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_compute_returns_zero_when_phase_disabled():
    module = _load_dist_pid_module()
    controller = module.DistPIDController(kp=1.0, ki=0.0, kd=0.0, dt=0.1, v_max=0.2)

    output = controller.compute(distance=0.6, desired_distance=0.3, phase_enabled=False)

    assert output == 0.0


def test_constructor_raises_for_non_positive_dt():
    module = _load_dist_pid_module()

    with pytest.raises(ValueError, match="dt must be > 0"):
        module.DistPIDController(kp=1.0, ki=0.0, kd=0.0, dt=0.0, v_max=0.2)


def test_deadband_outputs_zero_near_desired_distance():
    module = _load_dist_pid_module()
    controller = module.DistPIDController(
        kp=1.0,
        ki=0.0,
        kd=0.0,
        dt=0.1,
        v_max=0.2,
        distance_deadband=0.01,
    )

    output = controller.compute(distance=0.295, desired_distance=0.3, phase_enabled=True)

    assert output == 0.0


def test_separate_press_and_release_limits():
    module = _load_dist_pid_module()
    controller = module.DistPIDController(
        kp=10.0,
        ki=0.0,
        kd=0.0,
        dt=0.1,
        v_max=0.2,
        max_press_velocity=0.03,
        max_release_velocity=0.02,
    )

    press = controller.compute(distance=0.0, desired_distance=0.3, phase_enabled=True)
    controller.reset_integral()
    release = controller.compute(distance=0.6, desired_distance=0.3, phase_enabled=True)

    assert press == pytest.approx(0.03)
    assert release == pytest.approx(-0.02)


def test_slew_rate_limits_output_change():
    module = _load_dist_pid_module()
    controller = module.DistPIDController(
        kp=10.0,
        ki=0.0,
        kd=0.0,
        dt=0.1,
        v_max=0.2,
        normal_velocity_slew_rate=0.04,
    )

    first = controller.compute(distance=0.0, desired_distance=0.3, phase_enabled=True)
    second = controller.compute(distance=0.0, desired_distance=0.3, phase_enabled=True)

    assert first == pytest.approx(0.004)
    assert second == pytest.approx(0.008)


def test_distance_filter_smooths_raw_distance_step():
    module = _load_dist_pid_module()
    controller = module.DistPIDController(
        kp=1.0,
        ki=0.0,
        kd=0.0,
        dt=0.1,
        v_max=1.0,
        distance_filter_alpha=0.25,
    )

    controller.compute(distance=0.0, desired_distance=0.3, phase_enabled=True)
    controller.compute(distance=1.0, desired_distance=0.3, phase_enabled=True)

    assert controller.filtered_distance == pytest.approx(0.25)


def test_rc_switch_positive_runs_pid():
    module = _load_dist_pid_module()
    node = object.__new__(module.DistPIDControllerNode)
    node.controller = module.DistPIDController(kp=1.0, ki=0.0, kd=0.0, dt=0.1, v_max=0.2)
    node.phase_enabled = True
    node.distance = 0.1
    node.desired_distance = 0.2
    node.rc_switch = 1
    node.rc_retract_velocity = 0.03

    assert node._compute_rc_velocity() == pytest.approx(0.1)


def test_rc_switch_middle_resets_and_outputs_zero():
    module = _load_dist_pid_module()
    node = object.__new__(module.DistPIDControllerNode)
    node.controller = module.DistPIDController(kp=1.0, ki=1.0, kd=0.0, dt=0.1, v_max=0.2)
    node.phase_enabled = True
    node.distance = 0.1
    node.desired_distance = 0.2
    node.rc_switch = 0
    node.rc_retract_velocity = 0.03
    node.controller.integral = 1.0

    assert node._compute_rc_velocity() == 0.0
    assert node.controller.integral == 0.0


def test_rc_switch_negative_outputs_retract_velocity():
    module = _load_dist_pid_module()
    node = object.__new__(module.DistPIDControllerNode)
    node.controller = module.DistPIDController(kp=1.0, ki=0.0, kd=0.0, dt=0.1, v_max=0.2)
    node.phase_enabled = True
    node.distance = 0.1
    node.desired_distance = 0.2
    node.rc_switch = -1
    node.rc_retract_velocity = 0.03

    assert node._compute_rc_velocity() == pytest.approx(-0.03)
