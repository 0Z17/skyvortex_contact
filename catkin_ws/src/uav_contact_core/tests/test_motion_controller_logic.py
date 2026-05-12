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


def fallback_cmd(safety_unsafe, has_vel_ref, has_vel_normal, vel_ref_fresh, vel_normal_fresh, offboard_ready=True):
    inputs_ready = has_vel_ref and has_vel_normal and vel_ref_fresh and vel_normal_fresh
    if safety_unsafe or (not offboard_ready) or not inputs_ready:
        return [0.0, 0.0, 0.0]
    return [1.0, 1.0, 1.0]


def _controller_cpp_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "src"
        / "control"
        / "uav_motion_controller_node.cpp"
    )


def _motion_controller_config_path() -> Path:
    return (
        Path(__file__).resolve().parents[2]
        / "uav_contact_bringup"
        / "config"
        / "uav_motion_controller.yaml"
    )


def test_uav_motion_controller_cpp_exists():
    assert _controller_cpp_path().exists()


def test_default_publish_rate_is_50hz_in_cpp():
    content = _controller_cpp_path().read_text(encoding="utf-8")
    assert "constexpr double kDefaultPublishRateHz = 50.0;" in content


def test_velocity_fusion_and_clamp():
    v = fuse_cmd(v_ref=[0.3, 0.0, 0.0], v_normal=0.2, n=[1.0, 0.0, 0.0], vmax=0.25)

    assert v[0] == 0.25
    assert v[1] == 0.0
    assert v[2] == 0.0


def test_fallback_zero_when_safety_unsafe():
    v = fallback_cmd(
        safety_unsafe=True,
        has_vel_ref=True,
        has_vel_normal=True,
        vel_ref_fresh=True,
        vel_normal_fresh=True,
    )
    assert v == [0.0, 0.0, 0.0]


def test_fallback_zero_when_not_offboard_ready():
    v = fallback_cmd(
        safety_unsafe=False,
        has_vel_ref=True,
        has_vel_normal=True,
        vel_ref_fresh=True,
        vel_normal_fresh=True,
        offboard_ready=False,
    )
    assert v == [0.0, 0.0, 0.0]


def test_cpp_has_offboard_gate_param_and_subscriber():
    content = _controller_cpp_path().read_text(encoding="utf-8")
    assert "zero_when_not_offboard_ready" in content
    assert "\"/mavros/state\"" in content
    assert "msg->mode == \"OFFBOARD\"" in content


def test_cpp_has_tangent_decomposition_for_contact_phases():
    content = _controller_cpp_path().read_text(encoding="utf-8")
    assert "TangentialComponent" in content
    assert "max_tangent_velocity_" in content
    assert "normal_velocity = std::max(" in content
    assert "max_normal_velocity_" in content


def test_cpp_has_tangential_position_feedback_term():
    content = _controller_cpp_path().read_text(encoding="utf-8")
    assert "tangent_position_kp_" in content
    assert "p_ref_" in content and "p_meas_" in content
    assert "p_error_tangent" in content
    assert "\"/mavros/local_position/pose\"" in content


def test_cpp_uses_position_setpoint_for_retreat():
    content = _controller_cpp_path().read_text(encoding="utf-8")
    assert "retreat_distance_m_(0.3)" in content
    assert '"/trajectory_server/retreat_distance_m"' in content
    assert "CaptureRetreatPositionTarget()" in content
    assert "PublishRetreatPositionSetpoint(now)" in content
    assert "start[0] - retreat_distance * normal[0]" in content
    assert "msg.type_mask = kPositionOnlyTypeMask;" in content


def test_cpp_uses_reference_start_for_retreat_with_measured_pose_guard():
    content = _controller_cpp_path().read_text(encoding="utf-8")
    config = _motion_controller_config_path().read_text(encoding="utf-8")
    assert "retreat_start_max_deviation_m_(0.3)" in content
    assert '"retreat_start_max_deviation_m"' in content
    assert "if (!has_pose_meas_)" in content
    assert "deviation <= max_deviation" in content
    assert "start = p_ref_;" in content
    assert "falling back to measured pose" in content
    assert "retreat_start_max_deviation_m: 0.3" in config


def test_cpp_limits_position_setpoints_for_approach_and_retreat():
    content = _controller_cpp_path().read_text(encoding="utf-8")
    config = _motion_controller_config_path().read_text(encoding="utf-8")
    assert "LimitPositionTarget(" in content
    assert "LimitPositionTarget(p_ref_, approach_max_position_deviation_)" in content
    assert "LimitPositionTarget(retreat_position_target_" in content
    assert "retreat_max_position_deviation_m_(0.3)" in content
    assert '"retreat_max_position_deviation_m"' in content
    assert "retreat_max_position_deviation_m: 0.3" in config
