from pathlib import Path
import importlib.util


def _load_task_manager_module():
    module_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "task"
        / "task_manager_node.py"
    )
    spec = importlib.util.spec_from_file_location("task_manager_node", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_emergency_transition_override():
    module = _load_task_manager_module()
    manager = module.TaskManager()

    manager.state = "TRACKING"
    manager.on_safety_emergency()

    assert manager.state == "EMERGENCY_RETREAT"
