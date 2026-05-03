#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from uav_contact_msgs.msg import TaskPhase, ContactCommand


class DistPIDController:
    def __init__(self, kp, ki, kd, dt, v_max):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.dt = float(dt)
        if self.dt <= 0.0:
            raise ValueError("dt must be > 0")
        self.v_max = float(v_max)

        self.integral = 0.0
        self.prev_error = None

    def compute(self, distance, desired_distance, phase_enabled):
        if not phase_enabled:
            return 0.0

        error = float(distance) - float(desired_distance)

        p_term = self.kp * error
        self.integral += error * self.dt
        i_term = self.ki * self.integral

        if self.prev_error is None:
            d_term = 0.0
        else:
            d_term = self.kd * (error - self.prev_error) / self.dt

        output = p_term + i_term + d_term
        output = max(min(output, self.v_max), -self.v_max)

        self.prev_error = error
        return output

    def reset_integral(self):
        self.integral = 0.0
        self.prev_error = None


class DistPIDControllerNode:
    def __init__(self):
        rospy.init_node("dist_pid_controller", anonymous=False)

        kp = float(rospy.get_param("/contact_controller/pid/kp", 0.8))
        ki = float(rospy.get_param("/contact_controller/pid/ki", 0.0))
        kd = float(rospy.get_param("/contact_controller/pid/kd", 0.05))
        self.rate_hz = float(rospy.get_param("/contact_controller/rate", 50.0))
        self.v_max = float(rospy.get_param("/contact_controller/max_normal_velocity", 0.08))
        self.desired_distance = float(rospy.get_param("/contact_controller/desired_distance", 0.03))
        self.enabled_phases = rospy.get_param(
            "/contact_controller/enabled_phases",
            [TaskPhase.INITIAL_CONTACT, TaskPhase.SLIDING_CONTACT],
        )

        dt = 1.0 / max(self.rate_hz, 1.0)
        self.controller = DistPIDController(kp=kp, ki=ki, kd=kd, dt=dt, v_max=self.v_max)

        self.phase_enabled = False
        self.distance = 0.0

        self.cmd_pub = rospy.Publisher(
            "/uav_contact/contact/normal_velocity_cmd", ContactCommand, queue_size=10
        )

        self.phase_sub = rospy.Subscriber(
            "/uav_contact/task/phase", TaskPhase, self._on_task_phase
        )
        self.distance_sub = rospy.Subscriber(
            "/contact/distance", Float64, self._on_distance
        )

        rospy.loginfo("Dist PID controller started")

    def _on_task_phase(self, msg):
        newly_enabled = msg.phase in self.enabled_phases
        if not newly_enabled and self.phase_enabled:
            self.controller.reset_integral()
        self.phase_enabled = newly_enabled

    def _on_distance(self, msg):
        self.distance = msg.data

    def compute_and_publish(self):
        v_cmd = self.controller.compute(
            distance=self.distance,
            desired_distance=self.desired_distance,
            phase_enabled=self.phase_enabled,
        )

        msg = ContactCommand()
        msg.header.stamp = rospy.Time.now()
        msg.enabled = self.phase_enabled
        msg.normal_direction.x = 1.0
        msg.normal_direction.y = 0.0
        msg.normal_direction.z = 0.0
        msg.normal_velocity = v_cmd
        msg.normal_offset = 0.0
        msg.distance_error = self.controller.prev_error if self.controller.prev_error else 0.0
        msg.measured_distance = self.distance
        msg.desired_distance = self.desired_distance
        self.cmd_pub.publish(msg)

        return v_cmd

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.compute_and_publish()
            rate.sleep()


def main():
    node = DistPIDControllerNode()
    node.spin()


if __name__ == "__main__":
    main()
