#!/usr/bin/env python3

import os
import sys

try:
    import rospy
    from std_msgs.msg import Float32, Float64, Int8
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


def direction_and_magnitude_to_speed(direction, magnitude, max_speed, deadband=0.0, invert=False):
    direction = int(direction)
    if direction == 0:
        return 0.0
    magnitude = clamp(abs(float(magnitude)), 0.0, 1.0)
    if magnitude <= max(0.0, float(deadband)):
        return 0.0
    speed = float(direction) * magnitude * float(max_speed)
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
        self.direction_topic = param("direction_topic", "/uav_contact/rc/dynamixel_direction")
        self.speed_topic = param("speed_topic", "/uav_contact/rc/dynamixel_speed")
        self.joint_pos_topic = param("joint_pos_topic", "/joint_pos")
        self.max_speed = float(param("max_speed", 20.0))
        self.deadband = float(param("deadband", 0.02))
        self.invert = bool(param("invert", False))
        self.publish_rate_hz = float(param("publish_rate_hz", 50.0))
        self.command_timeout_sec = float(param("command_timeout_sec", 0.3))
        self.direction = 0
        self.magnitude = 0.0
        self.last_direction_time = rospy.Time(0)
        self.last_speed_time = rospy.Time(0)

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

        rospy.Subscriber(self.direction_topic, Int8, self._on_direction, queue_size=10)
        rospy.Subscriber(self.speed_topic, Float64, self._on_speed, queue_size=10)
        self.joint_pos_pub = rospy.Publisher(self.joint_pos_topic, Float32, queue_size=10)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(
            "RC to Dynamixel speed started: direction=%s speed=%s",
            self.direction_topic,
            self.speed_topic,
        )

    def _on_direction(self, msg):
        self.direction = int(msg.data)
        self.last_direction_time = rospy.Time.now()

    def _on_speed(self, msg):
        self.magnitude = float(msg.data)
        self.last_speed_time = rospy.Time.now()

    def current_speed(self):
        now = rospy.Time.now()
        direction_age = (now - self.last_direction_time).to_sec()
        speed_age = (now - self.last_speed_time).to_sec()
        if direction_age > self.command_timeout_sec or speed_age > self.command_timeout_sec:
            return 0.0
        return direction_and_magnitude_to_speed(
            self.direction,
            self.magnitude,
            self.max_speed,
            self.deadband,
            self.invert,
        )

    def publish_joint_position(self):
        try:
            self.joint_pos_pub.publish(Float32(data=float(self.controller.get_present_rad())))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Dynamixel position read failed: %s", exc)

    def on_shutdown(self):
        try:
            self.controller.set_vel(0.0)
        except Exception as exc:
            rospy.logwarn("Shutdown stop failed: %s", exc)

    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            self.controller.set_vel(self.current_speed())
            self.publish_joint_position()
            rate.sleep()


def main():
    node = RCToDynamixelSpeedNode()
    node.spin()


if __name__ == "__main__":
    main()
