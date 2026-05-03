from pathlib import Path
import importlib.util

import pytest


def _load_trajectory_module():
    module_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "trajectory"
        / "trajectory_server_node.py"
    )
    spec = importlib.util.spec_from_file_location("trajectory_server_node", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _make_publishers(module):
    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    return FakePublisher(), FakePublisher()


def _sample_waypoints():
    return [
        {
            "x": 1.0, "y": 2.0, "z": 3.0, "psi": 0.5, "theta": 0.25,
            "vx": 0.1, "vy": 0.2, "vz": 0.3, "vpsi": 0.4, "vtheta": 0.5,
            "nx": 0.6, "ny": 0.7, "nz": 0.8,
        },
        {
            "x": 4.0, "y": 5.0, "z": 6.0, "psi": 0.6, "theta": 0.3,
            "vx": 0.01, "vy": 0.02, "vz": 0.03, "vpsi": 0.04, "vtheta": 0.05,
            "nx": 0.1, "ny": 0.2, "nz": 0.3,
        },
    ]


def test_load_csv_returns_waypoints():
    module = _load_trajectory_module()
    server = module.TrajectoryServer()
    csv_path = Path(__file__).resolve().parents[1] / "data" / "exp_path.csv"

    waypoints = server.load_csv(csv_path)

    assert len(waypoints) > 3
    assert waypoints[0]["vx"] == 0.0
    assert waypoints[0]["vpsi"] == 0.0
    assert pytest.approx(waypoints[1]["x"], rel=1e-9) == 1.65286
    assert pytest.approx(waypoints[1]["vtheta"], rel=1e-9) == 0.0001759
    assert pytest.approx(waypoints[2]["nx"], rel=1e-9) == (
        module.math.cos(waypoints[2]["psi"]) * module.math.cos(waypoints[2]["theta"])
    )


def test_publish_stabilize_emits_hover_zero_velocity():
    module = _load_trajectory_module()
    traj_pub, joint_pub = _make_publishers(module)
    server = module.TrajectoryServer(
        trajectory_publisher=traj_pub,
        joint_publisher=joint_pub,
        publish_rate_hz=20.0,
    )
    server.waypoints = _sample_waypoints()
    server.phase = module.TaskPhase.STABILIZE

    server.publish()

    assert len(traj_pub.messages) == 1
    msg = traj_pub.messages[0]
    assert msg.vx == 0.0
    assert msg.vy == 0.0
    assert msg.vz == 0.0
    assert msg.vpsi == 0.0
    assert msg.vtheta == 0.0


def test_approach_builds_segment_from_stable_to_wp0_minus_offset_n():
    module = _load_trajectory_module()
    traj_pub, joint_pub = _make_publishers(module)
    server = module.TrajectoryServer(
        trajectory_publisher=traj_pub,
        joint_publisher=joint_pub,
        publish_rate_hz=10.0,
        approach_offset_m=0.3,
        approach_time_sec=0.3,
    )
    server.waypoints = _sample_waypoints()
    server._last_waypoint = server._make_waypoint(0.0, 0.0, 0.0, 0.5, 0.0)

    server.set_phase(module.TaskPhase.APPROACH)

    assert len(server.approach_path) == 3
    wp0 = server.waypoints[0]
    nx, ny, nz = server._normalized(wp0["nx"], wp0["ny"], wp0["nz"])
    end = server.approach_path[-1]
    assert pytest.approx(end["x"], rel=1e-9) == wp0["x"] - 0.3 * nx
    assert pytest.approx(end["y"], rel=1e-9) == wp0["y"] - 0.3 * ny
    assert pytest.approx(end["z"], rel=1e-9) == wp0["z"] - 0.3 * nz
    assert pytest.approx(end["theta"], rel=1e-9) == 0.0


def test_publish_approach_advances_segment_index_and_has_velocity():
    module = _load_trajectory_module()
    traj_pub, joint_pub = _make_publishers(module)
    server = module.TrajectoryServer(
        trajectory_publisher=traj_pub,
        joint_publisher=joint_pub,
        publish_rate_hz=10.0,
        approach_time_sec=0.3,
    )
    server.waypoints = _sample_waypoints()
    server._last_waypoint = server._make_waypoint(0.0, 0.0, 0.0, 0.5, 0.0)
    server.set_phase(module.TaskPhase.APPROACH)

    server.publish()
    server.publish()
    server.publish()

    assert server.approach_index == 2
    for msg in traj_pub.messages[-3:]:
        assert msg.vx != 0.0 or msg.vy != 0.0 or msg.vz != 0.0


def test_initial_contact_builds_segment_from_approach_end_to_wp0():
    module = _load_trajectory_module()
    traj_pub, joint_pub = _make_publishers(module)
    server = module.TrajectoryServer(
        trajectory_publisher=traj_pub,
        joint_publisher=joint_pub,
        publish_rate_hz=10.0,
        approach_offset_m=0.3,
        initial_contact_time_sec=0.3,
    )
    server.waypoints = _sample_waypoints()

    server.set_phase(module.TaskPhase.INITIAL_CONTACT)

    assert len(server.initial_contact_path) == 3
    start = server.initial_contact_path[0]
    end = server.initial_contact_path[-1]
    wp0 = server.waypoints[0]
    nx, ny, nz = server._normalized(wp0["nx"], wp0["ny"], wp0["nz"])
    assert pytest.approx(start["x"], rel=1e-9) == wp0["x"] - 0.3 * nx
    assert pytest.approx(start["y"], rel=1e-9) == wp0["y"] - 0.3 * ny
    assert pytest.approx(start["z"], rel=1e-9) == wp0["z"] - 0.3 * nz
    assert pytest.approx(end["x"], rel=1e-9) == wp0["x"]
    assert pytest.approx(end["y"], rel=1e-9) == wp0["y"]
    assert pytest.approx(end["z"], rel=1e-9) == wp0["z"]
    assert pytest.approx(end["theta"], rel=1e-9) == wp0["theta"]


def test_publish_initial_contact_advances_segment_index_and_has_velocity():
    module = _load_trajectory_module()
    traj_pub, joint_pub = _make_publishers(module)
    server = module.TrajectoryServer(
        trajectory_publisher=traj_pub,
        joint_publisher=joint_pub,
        publish_rate_hz=10.0,
        initial_contact_time_sec=0.3,
    )
    server.waypoints = _sample_waypoints()
    server.set_phase(module.TaskPhase.INITIAL_CONTACT)

    server.publish()
    server.publish()
    server.publish()

    assert server.initial_contact_index == 2
    for msg in traj_pub.messages[-3:]:
        assert msg.vx != 0.0 or msg.vy != 0.0 or msg.vz != 0.0


def test_sliding_builds_cost_interpolated_path():
    module = _load_trajectory_module()
    server = module.TrajectoryServer()
    server.waypoints = _sample_waypoints()

    server._build_sliding_path()

    assert len(server.sliding_path) > len(server.waypoints)
    assert pytest.approx(server.sliding_path[0]["x"], rel=1e-9) == server.waypoints[0]["x"]
    assert pytest.approx(server.sliding_path[-1]["x"], rel=1e-9) == server.waypoints[-1]["x"]


def test_publish_sliding_advances_over_cost_interpolated_path():
    module = _load_trajectory_module()
    traj_pub, joint_pub = _make_publishers(module)
    server = module.TrajectoryServer(
        trajectory_publisher=traj_pub,
        joint_publisher=joint_pub,
        publish_rate_hz=10.0,
    )
    server.waypoints = _sample_waypoints()
    server.set_phase(module.TaskPhase.SLIDING_CONTACT)

    server.publish()
    server.publish()
    server.publish()

    assert len(traj_pub.messages) == 3
    assert server.sliding_index >= 1


def test_publish_retreat_generates_leave_segment_and_zero_velocity():
    module = _load_trajectory_module()
    traj_pub, joint_pub = _make_publishers(module)
    server = module.TrajectoryServer(
        trajectory_publisher=traj_pub,
        joint_publisher=joint_pub,
        publish_rate_hz=10.0,
        leave_time_sec=0.3,
    )
    server.waypoints = _sample_waypoints()

    server.set_phase(module.TaskPhase.SLIDING_CONTACT)
    server.publish()
    server.publish()

    server.set_phase(module.TaskPhase.RETREAT)
    server.publish()
    server.publish()
    server.publish()

    assert len(server.retreat_path) == 3
    assert pytest.approx(server.retreat_path[-1]["x"], rel=1e-9) == server.retreat_path[0]["x"] - 0.5
    assert pytest.approx(server.retreat_path[-1]["theta"], rel=1e-9) == 0.0

    retreat_msgs = traj_pub.messages[-3:]
    for msg in retreat_msgs:
        assert msg.vx == 0.0
        assert msg.vy == 0.0
        assert msg.vz == 0.0
        assert msg.vpsi == 0.0
        assert msg.vtheta == 0.0


def test_load_csv_missing_file_raises():
    module = _load_trajectory_module()
    server = module.TrajectoryServer()

    with pytest.raises(FileNotFoundError, match="CSV file not found"):
        server.load_csv("/tmp/does-not-exist-task5.csv")


def test_load_csv_missing_required_header_raises(tmp_path):
    module = _load_trajectory_module()
    server = module.TrajectoryServer()
    csv_path = tmp_path / "missing_header.csv"
    csv_path.write_text("x,y,z,psi\n0,0,1,0\n", encoding="utf-8")

    with pytest.raises(ValueError, match="Missing required CSV columns"):
        server.load_csv(csv_path)


def test_load_csv_non_numeric_raises(tmp_path):
    module = _load_trajectory_module()
    server = module.TrajectoryServer()
    csv_path = tmp_path / "non_numeric.csv"
    csv_path.write_text("X,Y,Z,Psi,Theta\n0,abc,1,0,0\n", encoding="utf-8")

    with pytest.raises(ValueError, match="Non-numeric value"):
        server.load_csv(csv_path)
