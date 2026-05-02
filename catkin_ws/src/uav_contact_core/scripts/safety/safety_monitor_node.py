#!/usr/bin/env python3

"""Baseline safety monitor node with minimal attitude-limit evaluation."""


def evaluate_safety(roll_deg, pitch_deg, max_roll, max_pitch):
    if abs(roll_deg) > max_roll or abs(pitch_deg) > max_pitch:
        return "ATTITUDE_LIMIT_EXCEEDED"
    return "NORMAL"


class SafetyMonitorNode:
    def __init__(self, publisher=None, max_roll=12.0, max_pitch=12.0):
        self.publisher = publisher
        self.max_roll = max_roll
        self.max_pitch = max_pitch

    def evaluate_and_publish(self, roll_deg, pitch_deg):
        state = evaluate_safety(roll_deg, pitch_deg, self.max_roll, self.max_pitch)
        if self.publisher is not None:
            self.publisher.publish(state)
        return state


def main():
    try:
        import rospy
        from geometry_msgs.msg import Vector3
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError(
            "rospy, geometry_msgs, and std_msgs are required to run safety_monitor_node.py"
        ) from exc

    class _StringPublisherAdapter:
        def __init__(self, ros_publisher):
            self._ros_publisher = ros_publisher

        def publish(self, state):
            self._ros_publisher.publish(String(data=state))

    rospy.init_node("safety_monitor", anonymous=False)
    rospy.loginfo("Safety monitor baseline node started")

    max_roll = rospy.get_param("~max_roll_deg", 12.0)
    max_pitch = rospy.get_param("~max_pitch_deg", 12.0)
    state_publisher = rospy.Publisher("~state", String, queue_size=10)
    node = SafetyMonitorNode(
        publisher=_StringPublisherAdapter(state_publisher),
        max_roll=max_roll,
        max_pitch=max_pitch,
    )

    def _attitude_callback(msg):
        roll_deg = getattr(msg, "x", 0.0)
        pitch_deg = getattr(msg, "y", 0.0)
        node.evaluate_and_publish(roll_deg, pitch_deg)

    rospy.Subscriber("~attitude_deg", Vector3, _attitude_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
