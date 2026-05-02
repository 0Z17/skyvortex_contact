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


def test_load_csv_returns_waypoints():
    module = _load_trajectory_module()
    server = module.TrajectoryServer()
    csv_path = Path(__file__).resolve().parents[1] / "data" / "exp_path.csv"

    waypoints = server.load_csv(csv_path)

    assert waypoints == [
        (0.0, 0.0, 1.0, 0.0),
        (1.0, 0.5, 1.2, 0.1),
        (2.0, 1.0, 1.5, 0.2),
    ]


def test_load_csv_missing_file_raises():
    module = _load_trajectory_module()
    server = module.TrajectoryServer()

    with pytest.raises(FileNotFoundError, match="CSV file not found"):
        server.load_csv("/tmp/does-not-exist-task5.csv")


def test_load_csv_missing_required_header_raises(tmp_path):
    module = _load_trajectory_module()
    server = module.TrajectoryServer()
    csv_path = tmp_path / "missing_header.csv"
    csv_path.write_text("x,y,z\n0,0,1\n", encoding="utf-8")

    with pytest.raises(ValueError, match="Missing required CSV columns"):
        server.load_csv(csv_path)


def test_load_csv_non_numeric_raises(tmp_path):
    module = _load_trajectory_module()
    server = module.TrajectoryServer()
    csv_path = tmp_path / "non_numeric.csv"
    csv_path.write_text("x,y,z,yaw\n0,abc,1,0\n", encoding="utf-8")

    with pytest.raises(ValueError, match="Non-numeric value"):
        server.load_csv(csv_path)
