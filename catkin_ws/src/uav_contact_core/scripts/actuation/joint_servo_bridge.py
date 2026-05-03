#!/usr/bin/env python3

"""Baseline joint servo bridge node (skeleton)."""


def clamp_joint_angle(angle, joint_min, joint_max):
    return max(min(float(angle), float(joint_max)), float(joint_min))


def map_joint_angle_to_pwm(angle, joint_min, joint_max, pwm_min, pwm_max):
    joint_min = float(joint_min)
    joint_max = float(joint_max)
    if joint_max <= joint_min:
        raise ValueError("joint_max must be > joint_min")

    pwm_min = int(pwm_min)
    pwm_max = int(pwm_max)

    clamped = clamp_joint_angle(angle, joint_min, joint_max)
    normalized = (clamped - joint_min) / (joint_max - joint_min)
    pwm = pwm_min + normalized * (pwm_max - pwm_min)
    return int(round(pwm))


class JointServoBridge:
    def __init__(self, joint_min=-1.0, joint_max=1.0, pwm_min=1000, pwm_max=2000):
        self.node_name = "joint_servo_bridge"
        self.joint_min = float(joint_min)
        self.joint_max = float(joint_max)
        self.pwm_min = int(pwm_min)
        self.pwm_max = int(pwm_max)

    def joint_angle_to_pwm(self, angle):
        return map_joint_angle_to_pwm(
            angle,
            self.joint_min,
            self.joint_max,
            self.pwm_min,
            self.pwm_max,
        )


def main():
    try:
        import rospy
        from std_msgs.msg import Float64
    except ImportError as exc:
        raise RuntimeError("rospy and std_msgs are required to run joint_servo_bridge.py") from exc

    rospy.init_node("joint_servo_bridge", anonymous=False)
    publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 10.0))
    initial_pwm = float(rospy.get_param("~initial_pwm", 1500.0))
    publisher = rospy.Publisher("/servo/command", Float64, queue_size=10)

    rate = rospy.Rate(publish_rate_hz if publish_rate_hz > 0.0 else 10.0)
    while not rospy.is_shutdown():
        publisher.publish(Float64(data=initial_pwm))
        rate.sleep()


if __name__ == "__main__":
    main()
