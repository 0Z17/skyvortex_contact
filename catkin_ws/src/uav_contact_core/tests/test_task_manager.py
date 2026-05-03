from pathlib import Path
import importlib.util
import types


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


def test_initial_state_is_idle_constant():
    module = _load_task_manager_module()
    manager = module.TaskManager()

    assert manager.phase == module.TaskManager.IDLE


def test_emergency_transition_override():
    module = _load_task_manager_module()
    manager = module.TaskManager()

    manager.phase = module.TaskManager.APPROACH
    manager.on_safety_emergency()

    assert manager.phase == module.TaskManager.EMERGENCY_RETREAT


def test_emergency_transition_is_idempotent():
    module = _load_task_manager_module()
    manager = module.TaskManager()

    manager.on_safety_emergency()
    first_phase = manager.phase
    manager.on_safety_emergency()

    assert first_phase == module.TaskManager.EMERGENCY_RETREAT
    assert manager.phase == module.TaskManager.EMERGENCY_RETREAT


def _install_rospy_stub(module, params):
    class _Publisher:
        def __init__(self, *args, **kwargs):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    rospy_stub = types.SimpleNamespace(
        init_node=lambda *args, **kwargs: None,
        Publisher=lambda *args, **kwargs: _Publisher(),
        get_param=lambda name, default=None: params.get(name, default),
        Rate=lambda hz: types.SimpleNamespace(sleep=lambda: None),
        is_shutdown=lambda: True,
    )
    module.rospy = rospy_stub


def test_default_auto_start_sets_approach_phase():
    module = _load_task_manager_module()
    _install_rospy_stub(module, params={"/task_manager/publish_rate_hz": 10.0})

    node = module.TaskManagerNode()

    assert node.manager.phase == module.TaskPhase.PHASE_APPROACH
    assert node.manager.is_phase_enabled() is True


def test_invalid_initial_phase_falls_back_to_approach():
    module = _load_task_manager_module()
    _install_rospy_stub(
        module,
        params={
            "/task_manager/auto_start": True,
            "/task_manager/initial_phase": "NOT_A_REAL_PHASE",
            "/task_manager/publish_rate_hz": 10.0,
        },
    )

    node = module.TaskManagerNode()

    assert node.manager.phase == module.TaskPhase.PHASE_APPROACH
