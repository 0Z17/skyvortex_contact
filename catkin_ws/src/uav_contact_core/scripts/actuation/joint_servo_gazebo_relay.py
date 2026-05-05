#!/usr/bin/env python3

try:
    import rospy
    from std_msgs.msg import Float32, Float64
except ImportError:  # pragma: no cover
    rospy = None


def relay_param(name, default):
    return rospy.get_param(
        "/joint_servo_gazebo_relay/{}".format(name),
        rospy.get_param("~{}".format(name), default),
    )


class JointServoGazeboRelay:
    def __init__(self):
        if rospy is None:
            raise RuntimeError("rospy and std_msgs are required to run joint_servo_gazebo_relay.py")

        rospy.init_node("joint_servo_gazebo_relay", anonymous=False)

        self.enabled = bool(relay_param("enabled", False))
        self.input_topic = str(relay_param("input_topic", "/servo/command"))
        self.output_topic = str(relay_param("output_topic", "/skyvortex/operator_1_joint/pos_cmd"))
        self.joint_offset = float(relay_param("joint_offset", 0.0))

        self.pub = None
        self.sub = None
        if self.enabled:
            self.pub = rospy.Publisher(self.output_topic, Float32, queue_size=10)
            self.sub = rospy.Subscriber(self.input_topic, Float64, self._on_command, queue_size=10)
            rospy.loginfo(
                "Joint servo Gazebo relay enabled: %s -> %s, offset %.6f",
                self.input_topic,
                self.output_topic,
                self.joint_offset,
            )
        else:
            rospy.loginfo("Joint servo Gazebo relay disabled")

    def _on_command(self, msg):
        self.pub.publish(Float32(data=float(msg.data) + self.joint_offset))


if __name__ == "__main__":
    JointServoGazeboRelay()
    rospy.spin()
