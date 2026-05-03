#!/usr/bin/env python3

"""Baseline end-effector kinematics node (state-estimation scaffold only)."""


def build_state():
    return {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "contact_normal": {"x": 0.0, "y": 0.0, "z": 1.0},
        "estimated_wrench": {
            "force": {"x": 0.0, "y": 0.0, "z": 0.0},
            "torque": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
        "distance": 0.3,
        "in_contact": False,
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
        from geometry_msgs.msg import Pose, Twist, Vector3, Wrench
        from std_msgs.msg import Float64
        from uav_contact_msgs.msg import EndEffectorState
    except ImportError as exc:
        raise RuntimeError(
            "rospy, geometry_msgs, std_msgs, and uav_contact_msgs are required to run end_effector_kinematics_node.py"
        ) from exc

    class _EndEffectorPublisherAdapter:
        def __init__(self, ros_publisher, distance_publisher=None):
            self._ros_publisher = ros_publisher
            self._distance_publisher = distance_publisher

        def publish(self, state):
            msg = EndEffectorState()
            msg.header.stamp = rospy.Time.now()
            msg.pose = Pose()
            msg.pose.position.x = float(state["position"]["x"])
            msg.pose.position.y = float(state["position"]["y"])
            msg.pose.position.z = float(state["position"]["z"])
            msg.pose.orientation.x = float(state["orientation"]["x"])
            msg.pose.orientation.y = float(state["orientation"]["y"])
            msg.pose.orientation.z = float(state["orientation"]["z"])
            msg.pose.orientation.w = float(state["orientation"]["w"])
            msg.twist = Twist()
            msg.twist.linear.x = float(state["linear_velocity"]["x"])
            msg.twist.linear.y = float(state["linear_velocity"]["y"])
            msg.twist.linear.z = float(state["linear_velocity"]["z"])
            msg.twist.angular.x = float(state["angular_velocity"]["x"])
            msg.twist.angular.y = float(state["angular_velocity"]["y"])
            msg.twist.angular.z = float(state["angular_velocity"]["z"])
            msg.contact_normal = Vector3(
                x=float(state["contact_normal"]["x"]),
                y=float(state["contact_normal"]["y"]),
                z=float(state["contact_normal"]["z"]),
            )
            msg.estimated_wrench = Wrench()
            msg.estimated_wrench.force.x = float(state["estimated_wrench"]["force"]["x"])
            msg.estimated_wrench.force.y = float(state["estimated_wrench"]["force"]["y"])
            msg.estimated_wrench.force.z = float(state["estimated_wrench"]["force"]["z"])
            msg.estimated_wrench.torque.x = float(state["estimated_wrench"]["torque"]["x"])
            msg.estimated_wrench.torque.y = float(state["estimated_wrench"]["torque"]["y"])
            msg.estimated_wrench.torque.z = float(state["estimated_wrench"]["torque"]["z"])
            msg.in_contact = bool(state["in_contact"])
            self._ros_publisher.publish(msg)
            if self._distance_publisher is not None:
                self._distance_publisher.publish(Float64(data=float(state["distance"])))

    rospy.init_node("end_effector_kinematics", anonymous=False)
    rospy.loginfo("End-effector kinematics baseline node started")
    state_publisher = rospy.Publisher("/uav_contact/ee/state", EndEffectorState, queue_size=10)
    distance_publisher = rospy.Publisher("/uav_contact/distance", Float64, queue_size=10)
    node = EndEffectorKinematicsNode(
        publisher=_EndEffectorPublisherAdapter(state_publisher, distance_publisher)
    )
    publish_rate_hz = rospy.get_param("~publish_rate_hz", 10.0)
    rate = rospy.Rate(publish_rate_hz)

    while not rospy.is_shutdown():
        node.build_and_publish()
        rate.sleep()


if __name__ == "__main__":
    main()
