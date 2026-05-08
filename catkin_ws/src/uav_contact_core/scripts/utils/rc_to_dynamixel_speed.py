#!/usr/bin/env python3

import os
import sys

try:
    import rospy
    from std_msgs.msg import Float64
except ImportError:  # pragma: no cover
    rospy = None


def clamp(value, lower, upper):
    return max(min(float(value), float(upper)), float(lower))


def axis_to_speed(axis, max_speed, deadband=0.0, invert=False):
    axis = clamp(axis, -1.0, 1.0)
    if abs(axis) <= max(0.0, float(deadband)):
        return 0.0
    speed = axis * float(max_speed)
    return -speed if invert else speed


def param(name, default):
    return rospy.get_param(
        "/rc_to_dynamixel_speed/{}".format(name),
        rospy.get_param("~{}".format(name), default),
    )


class RCToDynamixelSpeedNode:
    def __init__(self):
        if rospy is None:
            raise RuntimeError("rospy and std_msgs are required")

        rospy.init_node("rc_to_dynamixel_speed", anonymous=False)
        self.input_topic = param("input_topic", "/uav_contact/rc/tangent_axis2")
        self.max_speed = float(param("max_speed", 20.0))
        self.deadband = float(param("deadband", 0.02))
        self.invert = bool(param("invert", False))
        self.publish_rate_hz = float(param("publish_rate_hz", 50.0))
        self.command_timeout_sec = float(param("command_timeout_sec", 0.3))
        self.axis = 0.0
        self.last_command_time = rospy.Time(0)

        actuation_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "actuation"))
        if actuation_dir not in sys.path:
            sys.path.insert(0, actuation_dir)
        from dynamixel_control import DynamixelController

        self.controller = DynamixelController(
            dxl_id=int(param("dxl_id", 1)),
            baudrate=int(param("dxl_baudrate", 57600)),
            devicename=str(param("dxl_devicename", "/dev/uav/joint_servo")),
        )
        self.controller.initialize()
        self.controller.set_operating_mode(int(param("dxl_operating_mode", 1)))

        rospy.Subscriber(self.input_topic, Float64, self._on_axis, queue_size=10)
        rospy.loginfo("RC to Dynamixel speed started: input=%s", self.input_topic)

    def _on_axis(self, msg):
        self.axis = float(msg.data)
        self.last_command_time = rospy.Time.now()

    def current_speed(self):
        age = (rospy.Time.now() - self.last_command_time).to_sec()
        if age > self.command_timeout_sec:
            return 0.0
        return axis_to_speed(self.axis, self.max_speed, self.deadband, self.invert)

    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            self.controller.set_vel(self.current_speed())
            rate.sleep()


def main():
    node = RCToDynamixelSpeedNode()
    node.spin()


if __name__ == "__main__":
    main()
