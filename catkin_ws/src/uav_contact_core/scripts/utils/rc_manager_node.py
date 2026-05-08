#!/usr/bin/env python3

try:
    import rospy
    from mavros_msgs.msg import RCIn
    from std_msgs.msg import Float64, Int8
except ImportError:  # pragma: no cover
    rospy = None

    class Float64:
        def __init__(self, data=0.0):
            self.data = float(data)

    class Int8:
        def __init__(self, data=0):
            self.data = int(data)


def clamp(value, lower, upper):
    return max(min(float(value), float(upper)), float(lower))


def map_pwm_to_axis(pwm, pwm_min, pwm_mid, pwm_max, deadband, invert=False):
    pwm = clamp(pwm, pwm_min, pwm_max)
    pwm_min = float(pwm_min)
    pwm_mid = float(pwm_mid)
    pwm_max = float(pwm_max)
    deadband = max(0.0, float(deadband))
    offset = pwm - pwm_mid

    if abs(offset) <= deadband:
        return 0.0

    if offset > 0.0:
        span = max(pwm_max - pwm_mid - deadband, 1.0)
        value = (offset - deadband) / span
    else:
        span = max(pwm_mid - pwm_min - deadband, 1.0)
        value = (offset + deadband) / span

    value = clamp(value, -1.0, 1.0)
    return -value if invert else value


def map_pwm_to_three_position(pwm, pwm_min, pwm_mid, pwm_max, deadband, invert=False):
    if pwm < pwm_min or pwm > pwm_max:
        return 0
    if abs(float(pwm) - float(pwm_mid)) <= float(deadband):
        return 0
    value = 1 if pwm < pwm_mid - deadband else -1
    return -value if invert else value


def param(name, default):
    return rospy.get_param("/rc_manager/{}".format(name), rospy.get_param("~{}".format(name), default))


class RCManagerNode:
    def __init__(self):
        if rospy is None:
            raise RuntimeError("rospy, mavros_msgs and std_msgs are required")

        rospy.init_node("rc_manager", anonymous=False)
        self.rc_topic = param("rc_topic", "/mavros/rc/in")
        self.axis1_channel = int(param("axis1_channel", 0))
        self.axis2_channel = int(param("axis2_channel", 1))
        self.normal_switch_channel = int(param("normal_switch_channel", 6))

        self.pwm_min = int(param("pwm_min", 1044))
        self.pwm_mid = int(param("pwm_mid", 1494))
        self.pwm_max = int(param("pwm_max", 1944))
        self.axis_deadband = int(param("axis_deadband", 50))
        self.switch_deadband = int(param("switch_deadband", 50))
        self.axis1_invert = bool(param("axis1_invert", False))
        self.axis2_invert = bool(param("axis2_invert", False))
        self.normal_switch_invert = bool(param("normal_switch_invert", False))

        self.tangent_axis1_topic = param("tangent_axis1_topic", "/uav_contact/rc/tangent_axis1")
        self.tangent_axis2_topic = param("tangent_axis2_topic", "/uav_contact/rc/tangent_axis2")
        self.normal_switch_topic = param("normal_switch_topic", "/uav_contact/rc/normal_switch")

        self.axis1_pub = rospy.Publisher(self.tangent_axis1_topic, Float64, queue_size=10)
        self.axis2_pub = rospy.Publisher(self.tangent_axis2_topic, Float64, queue_size=10)
        self.normal_switch_pub = rospy.Publisher(self.normal_switch_topic, Int8, queue_size=10)
        self.rc_sub = rospy.Subscriber(self.rc_topic, RCIn, self._on_rc, queue_size=10)

        rospy.loginfo("RC manager started: %s", self.rc_topic)

    def _channel(self, msg, index):
        if msg.channels and len(msg.channels) > index:
            return msg.channels[index]
        rospy.logwarn_throttle(1.0, "RC channel %d not available", index)
        return None

    def _on_rc(self, msg):
        axis1_pwm = self._channel(msg, self.axis1_channel)
        axis2_pwm = self._channel(msg, self.axis2_channel)
        switch_pwm = self._channel(msg, self.normal_switch_channel)

        axis1 = 0.0 if axis1_pwm is None else map_pwm_to_axis(
            axis1_pwm, self.pwm_min, self.pwm_mid, self.pwm_max,
            self.axis_deadband, self.axis1_invert,
        )
        axis2 = 0.0 if axis2_pwm is None else map_pwm_to_axis(
            axis2_pwm, self.pwm_min, self.pwm_mid, self.pwm_max,
            self.axis_deadband, self.axis2_invert,
        )
        switch = 0 if switch_pwm is None else map_pwm_to_three_position(
            switch_pwm, self.pwm_min, self.pwm_mid, self.pwm_max,
            self.switch_deadband, self.normal_switch_invert,
        )

        self.axis1_pub.publish(Float64(data=axis1))
        self.axis2_pub.publish(Float64(data=axis2))
        self.normal_switch_pub.publish(Int8(data=switch))


def main():
    node = RCManagerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
