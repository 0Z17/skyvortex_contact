#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64
from uav_contact_msgs.msg import TaskPhase, ContactCommand


def clamp(value, lower, upper):
    return max(min(float(value), float(upper)), float(lower))


class DistPIDController:
    def __init__(
        self,
        kp,
        ki,
        kd,
        dt,
        v_max,
        max_press_velocity=None,
        max_release_velocity=None,
        distance_filter_alpha=1.0,
        distance_deadband=0.0,
        normal_velocity_slew_rate=0.0,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.dt = float(dt)
        if self.dt <= 0.0:
            raise ValueError("dt must be > 0")
        self.v_max = float(v_max)
        self.max_press_velocity = float(
            self.v_max if max_press_velocity is None else max_press_velocity
        )
        self.max_release_velocity = float(
            self.v_max if max_release_velocity is None else max_release_velocity
        )
        self.distance_filter_alpha = clamp(distance_filter_alpha, 0.0, 1.0)
        self.distance_deadband = max(0.0, float(distance_deadband))
        self.normal_velocity_slew_rate = max(0.0, float(normal_velocity_slew_rate))

        self.integral = 0.0
        self.prev_error = None
        self.filtered_distance = None
        self.last_output = 0.0

    def compute(self, distance, desired_distance, phase_enabled):
        if not phase_enabled:
            self.reset_integral()
            return 0.0

        raw_distance = float(distance)
        if self.filtered_distance is None:
            self.filtered_distance = raw_distance
        else:
            alpha = self.distance_filter_alpha
            self.filtered_distance = (
                alpha * raw_distance + (1.0 - alpha) * self.filtered_distance
            )

        error = float(desired_distance) - self.filtered_distance
        if abs(error) <= self.distance_deadband:
            output = 0.0
            self.integral = 0.0
        else:
            p_term = self.kp * error
            self.integral += error * self.dt
            i_term = self.ki * self.integral

            if self.prev_error is None:
                d_term = 0.0
            else:
                d_term = self.kd * (error - self.prev_error) / self.dt

            output = p_term + i_term + d_term
            output = self._clamp_normal_velocity(output)

        output = self._apply_slew_rate(output)

        self.prev_error = error
        return output

    def _clamp_normal_velocity(self, output):
        output = clamp(output, -self.v_max, self.v_max)
        if output > 0.0:
            return min(output, self.max_press_velocity)
        if output < 0.0:
            return max(output, -self.max_release_velocity)
        return 0.0

    def _apply_slew_rate(self, output):
        if self.normal_velocity_slew_rate <= 0.0:
            self.last_output = output
            return output

        max_delta = self.normal_velocity_slew_rate * self.dt
        delta = output - self.last_output
        if delta > max_delta:
            output = self.last_output + max_delta
        elif delta < -max_delta:
            output = self.last_output - max_delta
        self.last_output = output
        return output

    def reset_integral(self):
        self.integral = 0.0
        self.prev_error = None
        self.filtered_distance = None
        self.last_output = 0.0


class DistPIDControllerNode:
    def __init__(self):
        rospy.init_node("dist_pid_controller", anonymous=False)

        kp = float(rospy.get_param("/contact_controller/pid/kp", 0.8))
        ki = float(rospy.get_param("/contact_controller/pid/ki", 0.0))
        kd = float(rospy.get_param("/contact_controller/pid/kd", 0.05))
        self.rate_hz = float(rospy.get_param("/contact_controller/rate", 50.0))
        self.v_max = float(rospy.get_param("/contact_controller/max_normal_velocity", 0.08))
        self.max_press_velocity = float(
            rospy.get_param("/contact_controller/max_press_velocity", self.v_max)
        )
        self.max_release_velocity = float(
            rospy.get_param("/contact_controller/max_release_velocity", self.v_max)
        )
        self.distance_filter_alpha = float(
            rospy.get_param("/contact_controller/distance_filter_alpha", 1.0)
        )
        self.distance_deadband = float(
            rospy.get_param("/contact_controller/distance_deadband", 0.0)
        )
        self.normal_velocity_slew_rate = float(
            rospy.get_param("/contact_controller/normal_velocity_slew_rate", 0.0)
        )
        self.desired_distance = float(rospy.get_param("/contact_controller/desired_distance", 0.03))
        self.enabled_phases = rospy.get_param(
            "/contact_controller/enabled_phases",
            [TaskPhase.INITIAL_CONTACT, TaskPhase.SLIDING_CONTACT],
        )
        self.normal_velocity_inhibit_topic = str(
            rospy.get_param(
                "/contact_controller/normal_velocity_inhibit_topic",
                "/uav_contact/safety/normal_velocity_inhibit",
            )
        )

        dt = 1.0 / max(self.rate_hz, 1.0)
        self.controller = DistPIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            dt=dt,
            v_max=self.v_max,
            max_press_velocity=self.max_press_velocity,
            max_release_velocity=self.max_release_velocity,
            distance_filter_alpha=self.distance_filter_alpha,
            distance_deadband=self.distance_deadband,
            normal_velocity_slew_rate=self.normal_velocity_slew_rate,
        )

        self.phase_enabled = False
        self.normal_velocity_inhibited = False
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
        self.normal_velocity_inhibit_sub = rospy.Subscriber(
            self.normal_velocity_inhibit_topic, Bool, self._on_normal_velocity_inhibit
        )

        rospy.loginfo("Dist PID controller started")

    def _on_task_phase(self, msg):
        newly_enabled = msg.phase in self.enabled_phases
        if not newly_enabled and self.phase_enabled:
            self.controller.reset_integral()
        self.phase_enabled = newly_enabled

    def _on_distance(self, msg):
        self.distance = msg.data

    def _on_normal_velocity_inhibit(self, msg):
        inhibited = bool(msg.data)
        if inhibited and not self.normal_velocity_inhibited:
            self.controller.reset_integral()
        self.normal_velocity_inhibited = inhibited

    def compute_and_publish(self):
        if self.normal_velocity_inhibited:
            self.controller.reset_integral()
            v_cmd = 0.0
        else:
            v_cmd = self.controller.compute(
                distance=self.distance,
                desired_distance=self.desired_distance,
                phase_enabled=self.phase_enabled,
            )

        msg = ContactCommand()
        msg.header.stamp = rospy.Time.now()
        msg.enabled = self.phase_enabled and not self.normal_velocity_inhibited
        msg.normal_velocity = v_cmd
        msg.normal_offset = 0.0
        msg.distance_error = self.controller.prev_error if self.controller.prev_error else 0.0
        msg.measured_distance = (
            self.controller.filtered_distance
            if self.controller.filtered_distance is not None
            else self.distance
        )
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
