from pathlib import Path
import importlib.util
import sys
import types

import pytest


def _load_dist_pid_module():
    class _FakeTime:
        @staticmethod
        def now():
            return 10.0

    published = []

    class _FakePublisher:
        def __init__(self, *args, **kwargs):
            self.published = published

        def publish(self, msg):
            self.published.append(msg)

    class _TaskPhase:
        INITIAL_CONTACT = 3
        SLIDING_CONTACT = 4

    class _ContactCommand:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)

    rospy_stub = types.SimpleNamespace(
        init_node=lambda *args, **kwargs: None,
        get_param=lambda name, default=None: default,
        Time=_FakeTime,
        Publisher=_FakePublisher,
        Subscriber=lambda *args, **kwargs: None,
        loginfo=lambda *args, **kwargs: None,
    )

    sys.modules["rospy"] = rospy_stub
    sys.modules["std_msgs.msg"] = types.SimpleNamespace(
        Bool=lambda data=False: types.SimpleNamespace(data=data),
        Float64=object,
    )
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
    module._test_published = published
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


def test_node_publishes_zero_normal_velocity_when_inhibited():
    module = _load_dist_pid_module()
    node = module.DistPIDControllerNode()
    node.phase_enabled = True
    node.distance = 0.0

    node.compute_and_publish()
    uninhibited_msg = module._test_published[-1]

    node._on_normal_velocity_inhibit(module.Bool(data=True))
    v_cmd = node.compute_and_publish()
    inhibited_msg = module._test_published[-1]

    assert uninhibited_msg.normal_velocity > 0.0
    assert v_cmd == 0.0
    assert inhibited_msg.normal_velocity == 0.0
    assert inhibited_msg.enabled is False
    assert node.phase_enabled is True
