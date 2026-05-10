#!/usr/bin/env python3

import math
import numpy as np

try:
    import rospy
    from geometry_msgs.msg import Twist
    from uav_contact_msgs.msg import TaskPhase, TrajectoryPoint
except ImportError:  # pragma: no cover
    rospy = None

    class _Obj:
        pass

    class Twist:
        def __init__(self):
            self.linear = _Obj()
            self.angular = _Obj()
            self.linear.x = 0.0
            self.linear.y = 0.0
            self.linear.z = 0.0
            self.angular.x = 0.0
            self.angular.y = 0.0
            self.angular.z = 0.0

    class TrajectoryPoint:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.psi = 0.0
            self.theta = 0.0
            self.vx = 0.0
            self.vy = 0.0
            self.vz = 0.0
            self.vpsi = 0.0
            self.vtheta = 0.0
            self.nx = 1.0
            self.ny = 0.0
            self.nz = 0.0

    class TaskPhase:
        APPROACH = 2
        SLIDING_CONTACT = 4


class EndEffectorTwistController:
    def __init__(self, link_length=0.7835, vel_factor=2.0, max_xy_speed=0.4):
        self.link_length = float(link_length)
        self.vel_factor = float(vel_factor)
        self.max_xy_speed = float(max_xy_speed)
        self.current_velocity = (0.0, 0.0, 0.0, 0.0, 0.0)
        self.current_config = None
        self.current_normal = (1.0, 0.0, 0.0)
        self.phase_enabled = False

    def update_trajectory_reference(self, msg):
        self.current_config = (
            float(msg.x),
            float(msg.y),
            float(msg.z),
            float(msg.psi),
            float(msg.theta),
        )
        self.current_velocity = (
            float(msg.vx),
            float(msg.vy),
            float(msg.vz),
            float(msg.vpsi),
            float(msg.vtheta),
        )
        self.current_normal = (float(msg.nx), float(msg.ny), float(msg.nz))

    def set_phase_enabled(self, enabled):
        self.phase_enabled = bool(enabled)

    def _jacobian(self, config):
        psi = config[3]
        theta = config[4]
        l = self.link_length
        return [
            [1.0, 0.0, 0.0, -l * math.cos(theta) * math.sin(psi), -l * math.cos(psi) * math.sin(theta)],
            [0.0, 1.0, 0.0,  l * math.cos(theta) * math.cos(psi),  -l * math.sin(theta) * math.sin(psi)],
            [0.0, 0.0, 1.0,  0.0,                                  -l * math.cos(theta)],
            [0.0, 0.0, 0.0,  1.0,                                   0.0],
            [0.0, 0.0, 0.0,  0.0,                                   1.0],
        ]

    @staticmethod
    def _normalize(v, fallback):
        vec = np.asarray(v, dtype=float)
        norm = np.linalg.norm(vec)
        if norm <= 1e-9:
            return np.asarray(fallback, dtype=float)
        return vec / norm

    def compute_twist(self):
        msg = Twist()
        if not self.phase_enabled or self.current_config is None:
            return msg

        vel_vec = np.array(self.current_velocity, dtype=float)
        jac = np.array(self._jacobian(self.current_config), dtype=float)
        vel_end = jac @ vel_vec

        n = self._normalize(self.current_normal, [1.0, 0.0, 0.0])
        ex = self._normalize(np.cross(np.array([0.0, 0.0, 1.0]), n), [1.0, 0.0, 0.0])
        ey = self._normalize(np.cross(n, ex), [0.0, 1.0, 0.0])

        vx = float(np.dot(vel_end[:3], ex)) * self.vel_factor
        vy = float(np.dot(vel_end[:3], ey)) * self.vel_factor

        vx = max(min(vx, self.max_xy_speed), -self.max_xy_speed)
        vy = max(min(vy, self.max_xy_speed), -self.max_xy_speed)

        msg.linear.x = -vx
        msg.linear.y = vy
        msg.linear.z = 0.0

        # print(f"Computed end-effector velocity: vx={vx:.3f}, vy={vy:.3f}, normal={n}")
        return msg


def main():
    if rospy is None:
        raise RuntimeError("rospy, geometry_msgs and uav_contact_msgs are required")

    rospy.init_node("end_effector_kinematics", anonymous=False)

    publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 50.0))
    controller = EndEffectorTwistController(
        link_length=float(rospy.get_param("~link_length", 0.7835)),
        vel_factor=float(rospy.get_param("~vel_factor", 2.0)),
        max_xy_speed=float(rospy.get_param("~max_xy_speed", 0.4)),
    )

    twist_pub = rospy.Publisher("/end_effector_velocity", Twist, queue_size=10)

    def _trajectory_ref_callback(msg):
        controller.update_trajectory_reference(msg)

    def _task_phase_callback(msg):
        controller.set_phase_enabled(
            msg.phase in (TaskPhase.APPROACH, TaskPhase.SLIDING_CONTACT)
        )

    rospy.Subscriber("/uav_contact/task/phase", TaskPhase, _task_phase_callback, queue_size=10)
    rospy.Subscriber("/uav_contact/trajectory/reference", TrajectoryPoint, _trajectory_ref_callback, queue_size=10)

    rate = rospy.Rate(publish_rate_hz)
    rospy.loginfo("End-effector twist controller started")

    while not rospy.is_shutdown():
        twist_pub.publish(controller.compute_twist())
        rate.sleep()


if __name__ == "__main__":
    main()
