from pathlib import Path
import importlib.util
import sys
import types


def _load_task_manager_module(params=None, now_sec=10.0):
    params = params or {}

    class _FakeDuration:
        def __init__(self, sec):
            self._sec = float(sec)

        def to_sec(self):
            return self._sec

    class _FakeTimeValue:
        def __init__(self, sec):
            self._sec = float(sec)

        def __sub__(self, other):
            return _FakeDuration(self._sec - other._sec)

    _clock = {"now": float(now_sec)}

    class _FakeTime:
        def __call__(self, sec=0.0):
            return _FakeTimeValue(sec)

        @staticmethod
        def now():
            return _FakeTimeValue(_clock["now"])

    rospy_stub = types.SimpleNamespace(
        init_node=lambda *args, **kwargs: None,
        get_param=lambda name, default=None: params.get(name, default),
        Time=_FakeTime(),
        Publisher=lambda *args, **kwargs: types.SimpleNamespace(publish=lambda msg: None),
        Subscriber=lambda *args, **kwargs: None,
        Rate=lambda hz: types.SimpleNamespace(sleep=lambda: None),
        is_shutdown=lambda: True,
        loginfo=lambda *args, **kwargs: None,
        logwarn=lambda *args, **kwargs: None,
    )

    class _TaskPhaseMsg:
        IDLE = 0
        STABILIZE = 1
        APPROACH = 2
        INITIAL_CONTACT = 3
        SLIDING_CONTACT = 4
        RETREAT = 5
        EMERGENCY_RETREAT = 6
        FINISHED = 7
        ERROR = 8

        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)
            self.phase = _TaskPhaseMsg.IDLE
            self.elapsed_time = 0.0
            self.enable_trajectory = False
            self.enable_contact_control = False
            self.enable_servo = False
            self.enable_uav_control = False
            self.description = ""

    class _SafetyStateMsg:
        def __init__(self, safe=True, require_emergency_retreat=False, reason=""):
            self.safe = safe
            self.require_emergency_retreat = require_emergency_retreat
            self.reason = reason

    class _BoolMsg:
        def __init__(self, data=False):
            self.data = bool(data)

    sys.modules["rospy"] = rospy_stub
    sys.modules["std_msgs.msg"] = types.SimpleNamespace(Bool=_BoolMsg)
    sys.modules["uav_contact_msgs.msg"] = types.SimpleNamespace(
        TaskPhase=_TaskPhaseMsg,
        SafetyState=_SafetyStateMsg,
    )

    module_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "task"
        / "task_manager_node.py"
    )
    spec = importlib.util.spec_from_file_location("task_manager_node", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module._test_clock = _clock
    return module


def test_initial_phase_is_idle():
    module = _load_task_manager_module()
    node = module.TaskManagerNode()
    assert node.phase == module.TaskPhase.IDLE


def test_auto_start_blocked_when_offboard_not_ready():
    module = _load_task_manager_module({"/task_manager/auto_start": True, "/task_manager/gate_on_offboard_ready": True})
    node = module.TaskManagerNode()

    node._update_phase()

    assert node.phase == module.TaskPhase.IDLE


def test_auto_start_transitions_when_offboard_ready_for_hold_time():
    module = _load_task_manager_module(
        {
            "/task_manager/auto_start": True,
            "/task_manager/gate_on_offboard_ready": True,
            "/task_manager/offboard_ready_hold_sec": 0.5,
        }
    )
    node = module.TaskManagerNode()

    safe_msg = module.SafetyState(safe=True, require_emergency_retreat=False, reason="")
    node._on_safety_state(safe_msg)

    module._test_clock["now"] += 0.6
    node._update_phase()

    assert node.phase == module.TaskPhase.STABILIZE


def test_transition_to_approach_requires_offboard_ready():
    module = _load_task_manager_module(
        {
            "/task_manager/gate_on_offboard_ready": True,
            "/task_manager/stabilize_duration": 0.1,
        }
    )
    node = module.TaskManagerNode()
    node.phase = module.TaskPhase.STABILIZE
    node.phase_start_time = module.rospy.Time.now()

    module._test_clock["now"] += 0.2
    node._update_phase()
    assert node.phase == module.TaskPhase.STABILIZE

    safe_msg = module.SafetyState(safe=True, require_emergency_retreat=False, reason="")
    node._on_safety_state(safe_msg)
    module._test_clock["now"] += 0.6
    node._update_phase()

    assert node.phase == module.TaskPhase.APPROACH


def test_sliding_done_transitions_to_retreat():
    module = _load_task_manager_module()
    node = module.TaskManagerNode()
    node.phase = module.TaskPhase.SLIDING_CONTACT

    node._on_sliding_done(module.Bool(data=True))
    node._update_phase()

    assert node.phase == module.TaskPhase.RETREAT


def test_semi_auto_stabilize_transitions_directly_to_sliding():
    module = _load_task_manager_module(
        {
            "/task_manager/semi_auto_mode": True,
            "/task_manager/gate_on_offboard_ready": True,
            "/task_manager/offboard_ready_hold_sec": 0.5,
            "/task_manager/stabilize_duration": 0.2,
        }
    )
    node = module.TaskManagerNode()

    node._on_safety_state(module.SafetyState(safe=True, require_emergency_retreat=False, reason=""))
    module._test_clock["now"] += 0.6
    node._update_phase()
    assert node.phase == module.TaskPhase.STABILIZE

    module._test_clock["now"] += 0.3
    node._update_phase()
    assert node.phase == module.TaskPhase.SLIDING_CONTACT


def test_semi_auto_ignores_sliding_done():
    module = _load_task_manager_module({"/task_manager/semi_auto_mode": True})
    node = module.TaskManagerNode()
    node.phase = module.TaskPhase.SLIDING_CONTACT

    node._on_sliding_done(module.Bool(data=True))
    node._update_phase()

    assert node.phase == module.TaskPhase.SLIDING_CONTACT


def test_emergency_request_forces_emergency_retreat():
    module = _load_task_manager_module()
    node = module.TaskManagerNode()
    node.phase = module.TaskPhase.APPROACH

    emergency_msg = module.SafetyState(safe=False, require_emergency_retreat=True, reason="OFFBOARD_DROPPED")
    node._on_safety_state(emergency_msg)

    assert node.phase == module.TaskPhase.EMERGENCY_RETREAT
