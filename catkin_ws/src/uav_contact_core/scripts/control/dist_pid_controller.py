#!/usr/bin/env python3

try:
    import rospy
    from std_msgs.msg import Bool, Float64
except ImportError:  # pragma: no cover - allows tests without ROS runtime
    rospy = None
    Bool = None
    Float64 = None


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

    def compute(self, distance, target_distance, phase_enabled):
        if not phase_enabled:
            return 0.0

        error = float(distance) - float(target_distance)

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


class DistPIDControllerNodeCore:
    """ROS-agnostic logic wrapper to allow unit tests without rospy."""

    def __init__(self, controller, velocity_publisher, target_distance):
        self.controller = controller
        self._velocity_publisher = velocity_publisher
        self.target_distance = float(target_distance)
        self.phase_enabled = False
        self.distance = None

    def set_phase_enabled(self, enabled):
        self.phase_enabled = bool(enabled)

    def set_distance(self, distance):
        self.distance = float(distance)

    def compute_and_publish(self):
        if self.distance is None:
            return None

        v_cmd = self.controller.compute(
            distance=self.distance,
            target_distance=self.target_distance,
            phase_enabled=self.phase_enabled,
        )
        self._velocity_publisher.publish(v_cmd)
        return v_cmd


class DistPIDControllerNode:
    def __init__(self):
        if rospy is None:
            raise RuntimeError("rospy is required to run DistPIDControllerNode")

        rospy.init_node("dist_pid_controller", anonymous=False)

        kp = float(rospy.get_param("/contact_controller/kp", 0.8))
        ki = float(rospy.get_param("/contact_controller/ki", 0.0))
        kd = float(rospy.get_param("/contact_controller/kd", 0.0))
        dt = float(rospy.get_param("/contact_controller/dt", 0.02))
        v_max = float(rospy.get_param("/contact_controller/v_max", 0.25))
        target_distance = float(rospy.get_param("/contact_controller/target_distance", 0.3))
        self.publish_rate_hz = float(rospy.get_param("/contact_controller/publish_rate_hz", 50.0))

        velocity_topic = rospy.get_param("/topics/velocity_normal_cmd", "/uav_contact/velocity_normal_cmd")
        phase_enabled_topic = rospy.get_param("~phase_enabled_topic", "/uav_contact/phase_enabled")
        distance_topic = rospy.get_param("~distance_topic", "/uav_contact/distance")

        self.velocity_pub = rospy.Publisher(velocity_topic, Float64, queue_size=10)
        self.core = DistPIDControllerNodeCore(
            controller=DistPIDController(kp=kp, ki=ki, kd=kd, dt=dt, v_max=v_max),
            velocity_publisher=self,
            target_distance=target_distance,
        )

        self._phase_sub = rospy.Subscriber(phase_enabled_topic, Bool, self._on_phase_enabled)
        self._distance_sub = rospy.Subscriber(distance_topic, Float64, self._on_distance)

    def publish(self, v_cmd):
        self.velocity_pub.publish(Float64(data=float(v_cmd)))

    def _on_phase_enabled(self, msg):
        self.core.set_phase_enabled(msg.data)

    def _on_distance(self, msg):
        self.core.set_distance(msg.data)

    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            self.core.compute_and_publish()
            rate.sleep()


def main():
    if rospy is None:
        raise RuntimeError("rospy is not available")
    node = DistPIDControllerNode()
    node.spin()


if __name__ == "__main__":
    main()
