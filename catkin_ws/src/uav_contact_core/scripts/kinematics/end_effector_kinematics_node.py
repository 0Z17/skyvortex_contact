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
    import rospy

    rospy.init_node("end_effector_kinematics", anonymous=False)
    rospy.loginfo("End-effector kinematics baseline node started")
    node = EndEffectorKinematicsNode()
    node.build_and_publish()
    rospy.spin()


if __name__ == "__main__":
    main()
