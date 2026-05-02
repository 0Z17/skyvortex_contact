#!/usr/bin/env python3

"""Baseline end-effector kinematics node (state-estimation scaffold only)."""


def build_state():
    return {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "normal_velocity": 0.0,
        "contact_error": 0.0,
    }


class EndEffectorKinematicsNode:
    def __init__(self, publisher=None):
        self.publisher = publisher

    def build_and_publish(self):
        state = build_state()
        if self.publisher is not None:
            self.publisher.publish(state)
        return state


def main():
    try:
        import rospy
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError(
            "rospy and std_msgs are required to run end_effector_kinematics_node.py"
        ) from exc

    class _StringPublisherAdapter:
        def __init__(self, ros_publisher):
            self._ros_publisher = ros_publisher

        def publish(self, state):
            self._ros_publisher.publish(String(data=str(state)))

    rospy.init_node("end_effector_kinematics", anonymous=False)
    rospy.loginfo("End-effector kinematics baseline node started")
    state_publisher = rospy.Publisher("~state", String, queue_size=10)
    node = EndEffectorKinematicsNode(publisher=_StringPublisherAdapter(state_publisher))
    publish_rate_hz = rospy.get_param("~publish_rate_hz", 10.0)
    rate = rospy.Rate(publish_rate_hz)

    while not rospy.is_shutdown():
        node.build_and_publish()
        rate.sleep()


if __name__ == "__main__":
    main()
