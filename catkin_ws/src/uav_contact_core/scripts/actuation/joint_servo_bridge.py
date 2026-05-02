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
    bridge = JointServoBridge()
    return bridge


if __name__ == "__main__":
    main()
