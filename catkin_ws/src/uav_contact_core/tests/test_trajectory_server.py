from pathlib import Path
import importlib.util


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
