#!/usr/bin/env python3

"""Baseline safety monitor node with minimal attitude-limit evaluation."""


def evaluate_safety(roll_deg, pitch_deg, max_roll, max_pitch):
    if abs(roll_deg) > max_roll or abs(pitch_deg) > max_pitch:
        return False, "ATTITUDE_LIMIT_EXCEEDED"
    return True, "NORMAL"


class SafetyMonitorNode:
    def __init__(self, publisher=None, max_roll=12.0, max_pitch=12.0):
        self.publisher = publisher
        self.max_roll = max_roll
        self.max_pitch = max_pitch

    def evaluate_and_publish(self, roll_deg, pitch_deg):
        armed, reason = evaluate_safety(roll_deg, pitch_deg, self.max_roll, self.max_pitch)
        if self.publisher is not None:
            self.publisher.publish(armed, reason)
        return armed, reason


def main():
    try:
        import rospy
        from geometry_msgs.msg import Vector3
        from uav_contact_msgs.msg import SafetyState
    except ImportError as exc:
        raise RuntimeError(
            "rospy, geometry_msgs, and uav_contact_msgs are required to run safety_monitor_node.py"
        ) from exc

    class _SafetyPublisherAdapter:
        def __init__(self, ros_publisher):
            self._ros_publisher = ros_publisher

        def publish(self, armed, reason):
            msg = SafetyState()
            msg.header.stamp = rospy.Time.now()
            msg.armed = bool(armed)
            msg.e_stop = not bool(armed)
            msg.watchdog_ok = True
            msg.geofence_ok = True
            msg.contact_force_ok = True
            msg.reason = reason
            self._ros_publisher.publish(msg)

    rospy.init_node("safety_monitor", anonymous=False)
    rospy.loginfo("Safety monitor baseline node started")

    max_roll = rospy.get_param("~max_roll_deg", 12.0)
    max_pitch = rospy.get_param("~max_pitch_deg", 12.0)
    state_publisher = rospy.Publisher("~state", SafetyState, queue_size=10)
    node = SafetyMonitorNode(
        publisher=_SafetyPublisherAdapter(state_publisher),
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
