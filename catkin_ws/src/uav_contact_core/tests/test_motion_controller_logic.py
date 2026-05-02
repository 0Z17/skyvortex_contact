from pathlib import Path
import math


def fuse_cmd(v_ref, v_normal, n, vmax):
    v_cmd = [
        float(v_ref[0]) + float(v_normal) * float(n[0]),
        float(v_ref[1]) + float(v_normal) * float(n[1]),
        float(v_ref[2]) + float(v_normal) * float(n[2]),
    ]
    norm = math.sqrt(v_cmd[0] ** 2 + v_cmd[1] ** 2 + v_cmd[2] ** 2)
    if norm > vmax and norm > 0.0:
        scale = vmax / norm
        v_cmd = [component * scale for component in v_cmd]
    return v_cmd


def _controller_cpp_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "src"
        / "control"
        / "uav_motion_controller_node.cpp"
    )


def test_uav_motion_controller_cpp_exists():
    assert _controller_cpp_path().exists()


def test_velocity_fusion_and_clamp():
    v = fuse_cmd(v_ref=[0.3, 0.0, 0.0], v_normal=0.2, n=[1.0, 0.0, 0.0], vmax=0.25)

    assert v[0] == 0.25
    assert v[1] == 0.0
    assert v[2] == 0.0
