from pathlib import Path
import importlib.util

import pytest


def _load_dist_pid_module():
    module_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "control"
        / "dist_pid_controller.py"
    )
    spec = importlib.util.spec_from_file_location("dist_pid_controller", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_compute_returns_zero_when_phase_disabled():
    module = _load_dist_pid_module()
    controller = module.DistPIDController(kp=1.0, ki=0.0, kd=0.0, dt=0.1, v_max=0.2)

    output = controller.compute(distance=0.6, target_distance=0.3, phase_enabled=False)

    assert output == 0.0


def test_constructor_raises_for_non_positive_dt():
    module = _load_dist_pid_module()

    with pytest.raises(ValueError, match="dt must be > 0"):
        module.DistPIDController(kp=1.0, ki=0.0, kd=0.0, dt=0.0, v_max=0.2)
