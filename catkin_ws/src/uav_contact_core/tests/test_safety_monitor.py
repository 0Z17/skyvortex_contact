from pathlib import Path
import importlib.util
import sys
import types


def _load_safety_module():
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

    class _FakeTime:
        now_sec = 10.0

        def __call__(self, sec=0.0):
            return _FakeTimeValue(sec)

        @staticmethod
        def now():
            return _FakeTimeValue(_FakeTime.now_sec)

    rospy_stub = types.SimpleNamespace(
        init_node=lambda *args, **kwargs: None,
        get_param=lambda name, default=None: default,
        Time=_FakeTime(),
        Publisher=lambda *args, **kwargs: types.SimpleNamespace(publish=lambda msg: None),
        Subscriber=lambda *args, **kwargs: None,
        Rate=lambda hz: types.SimpleNamespace(sleep=lambda: None),
        is_shutdown=lambda: True,
        loginfo=lambda *args, **kwargs: None,
    )

    class _TaskPhase:
        IDLE = 0
        APPROACH = 2
        INITIAL_CONTACT = 3
        SLIDING_CONTACT = 4
        RETREAT = 5

    class _SafetyState:
        NORMAL = 0
        MAVROS_DISCONNECTED = 1
        SENSOR_TIMEOUT = 2
        ATTITUDE_LIMIT_EXCEEDED = 3
        CONTACT_LOSS = 4
        DISTANCE_JUMP = 5
        EMERGENCY_RETREAT_REQUIRED = 6

    sys.modules["rospy"] = rospy_stub
    sys.modules["geometry_msgs.msg"] = types.SimpleNamespace(PoseStamped=object, Vector3=object)
    sys.modules["sensor_msgs.msg"] = types.SimpleNamespace(Imu=object)
    sys.modules["mavros_msgs.msg"] = types.SimpleNamespace(State=object)
    sys.modules["std_msgs.msg"] = types.SimpleNamespace(Float64=object)
    sys.modules["uav_contact_msgs.msg"] = types.SimpleNamespace(SafetyState=_SafetyState, TaskPhase=_TaskPhase)

    module_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "safety"
        / "safety_monitor_node.py"
    )
    spec = importlib.util.spec_from_file_location("safety_monitor_node", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_not_offboard_is_unsafe():
    module = _load_safety_module()
    node = module.SafetyMonitorNode()
    node.last_imu_time = module.rospy.Time.now()
    node.last_mavros_state_time = module.rospy.Time.now()
    node.mavros_connected = True
    node.mavros_mode = "POSCTL"
    node.mavros_armed = True

    state, safe, emergency, reason = node.evaluate()

    assert safe is False
    assert state == module.SafetyState.MAVROS_DISCONNECTED
    assert "NOT_OFFBOARD" in reason
    assert emergency is False


def test_offboard_drop_in_active_phase_requests_emergency():
    module = _load_safety_module()
    node = module.SafetyMonitorNode()
    node.last_imu_time = module.rospy.Time.now()
    node.last_mavros_state_time = module.rospy.Time.now()
    node.mavros_connected = True
    node.mavros_mode = "POSCTL"
    node.mavros_armed = True
    node.current_phase = module.TaskPhase.APPROACH

    state, safe, emergency, _ = node.evaluate()

    assert safe is False
    assert emergency is True
    assert state == module.SafetyState.EMERGENCY_RETREAT_REQUIRED
