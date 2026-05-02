#!/usr/bin/env python3

try:
    import rospy
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - allows tests without ROS runtime
    rospy = None
    String = None


class TaskManager:
    """Minimal Task 4 baseline task manager with explicit phases."""

    # Task 4 local state-machine phase constants for baseline behavior.
    # Message-level enum integration is intentionally handled in later integration tasks.
    IDLE = "IDLE"
    STABILIZE = "STABILIZE"
    APPROACH = "APPROACH"
    INITIAL_CONTACT = "INITIAL_CONTACT"
    SLIDING_CONTACT = "SLIDING_CONTACT"
    RETREAT = "RETREAT"
    EMERGENCY_RETREAT = "EMERGENCY_RETREAT"
    FINISHED = "FINISHED"
    ERROR = "ERROR"

    def __init__(self, phase_publisher=None):
        self.phase = self.IDLE
        self._phase_publisher = phase_publisher

    def publish_phase(self):
        if self._phase_publisher is None:
            return
        self._phase_publisher.publish(self.phase)

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
        self.phase_pub = rospy.Publisher("~task_phase", String, queue_size=10)
        self.manager = TaskManager(phase_publisher=self.phase_pub)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.manager.publish_phase()
            rate.sleep()


if __name__ == "__main__":
    if rospy is None:
        raise RuntimeError("rospy is not available")
    node = TaskManagerNode()
    node.spin()
