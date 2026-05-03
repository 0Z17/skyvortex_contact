#!/usr/bin/env python3

try:
    import rospy
    from std_msgs.msg import Bool
    from uav_contact_msgs.msg import TaskPhase
except ImportError:  # pragma: no cover - allows tests without ROS runtime
    rospy = None

    class Bool:
        def __init__(self, data=False):
            self.data = bool(data)

    class TaskPhase:  # minimal fallback for non-ROS test environments
        PHASE_IDLE = "IDLE"
        PHASE_APPROACH = "APPROACH"
        PHASE_ALIGN = "ALIGN"
        PHASE_CONTACT = "CONTACT"
        PHASE_TRACK = "TRACK"
        PHASE_RELEASE = "RELEASE"
        PHASE_ABORT = "ABORT"

        def __init__(self):
            self.phase = self.PHASE_IDLE
            self.progress = 0.0
            self.active_constraints = []


class TaskManager:
    """Minimal Task 4 baseline task manager with explicit phases."""

    PHASES_ENABLED_FOR_CONTACT_CONTROL = {
        TaskPhase.PHASE_APPROACH,
        TaskPhase.PHASE_ALIGN,
        TaskPhase.PHASE_CONTACT,
        TaskPhase.PHASE_TRACK,
    }

    PHASE_SEQUENCE = [
        TaskPhase.PHASE_IDLE,
        TaskPhase.PHASE_APPROACH,
        TaskPhase.PHASE_ALIGN,
        TaskPhase.PHASE_CONTACT,
        TaskPhase.PHASE_TRACK,
        TaskPhase.PHASE_RELEASE,
        TaskPhase.PHASE_ABORT,
    ]

    # Align local state values with TaskPhase message constants.
    IDLE = TaskPhase.PHASE_IDLE
    STABILIZE = TaskPhase.PHASE_ALIGN
    APPROACH = TaskPhase.PHASE_APPROACH
    INITIAL_CONTACT = TaskPhase.PHASE_CONTACT
    SLIDING_CONTACT = TaskPhase.PHASE_TRACK
    RETREAT = TaskPhase.PHASE_RELEASE
    EMERGENCY_RETREAT = TaskPhase.PHASE_ABORT
    FINISHED = TaskPhase.PHASE_IDLE
    ERROR = TaskPhase.PHASE_ABORT

    def __init__(self, phase_publisher=None):
        self.phase = self.IDLE
        self._phase_publisher = phase_publisher

    def publish_phase(self):
        if self._phase_publisher is None:
            return
        msg = TaskPhase()
        msg.phase = self.phase
        msg.progress = 0.0
        msg.active_constraints = []
        self._phase_publisher.publish(msg)

    def is_phase_enabled(self):
        return self.phase in self.PHASES_ENABLED_FOR_CONTACT_CONTROL

    def on_safety_emergency(self):
        if self.phase != self.EMERGENCY_RETREAT:
            self.phase = self.EMERGENCY_RETREAT
        self.publish_phase()


class TaskManagerNode:
    """Minimal ROS node scaffold for Task 4 baseline behavior."""

    def __init__(self):
        if rospy is None:
            raise RuntimeError("rospy is required to run TaskManagerNode")

        rospy.init_node("task_manager_node", anonymous=False)
        self.phase_pub = rospy.Publisher("/uav_contact/task_phase", TaskPhase, queue_size=10)
        self.phase_enabled_pub = rospy.Publisher("/uav_contact/phase_enabled", Bool, queue_size=10)
        self.manager = TaskManager(phase_publisher=self.phase_pub)
        self.publish_rate_hz = float(rospy.get_param("/task_manager/publish_rate_hz", 10.0))

        auto_start = bool(rospy.get_param("/task_manager/auto_start", True))
        initial_phase_param = rospy.get_param("/task_manager/initial_phase", TaskPhase.PHASE_APPROACH)
        if auto_start:
            self.manager.phase = self._resolve_initial_phase(initial_phase_param)

    def _resolve_initial_phase(self, initial_phase_param):
        if isinstance(initial_phase_param, int):
            if 0 <= initial_phase_param < len(TaskManager.PHASE_SEQUENCE):
                return TaskManager.PHASE_SEQUENCE[initial_phase_param]
            return TaskPhase.PHASE_APPROACH

        candidate = str(initial_phase_param).strip().upper()
        if not candidate.startswith("PHASE_"):
            candidate = f"PHASE_{candidate}"

        resolved = getattr(TaskPhase, candidate, None)
        if resolved in TaskManager.PHASE_SEQUENCE:
            return resolved

        return TaskPhase.PHASE_APPROACH

    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            self.manager.publish_phase()
            self.phase_enabled_pub.publish(Bool(data=self.manager.is_phase_enabled()))
            rate.sleep()


if __name__ == "__main__":
    if rospy is None:
        raise RuntimeError("rospy is not available")
    node = TaskManagerNode()
    node.spin()
