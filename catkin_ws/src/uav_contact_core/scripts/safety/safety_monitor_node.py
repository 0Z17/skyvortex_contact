#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ActuatorControl, State as MavrosState
from std_msgs.msg import Float64
from uav_contact_msgs.msg import SafetyState, TaskPhase


class SafetyMonitorNode:
    def __init__(self):
        rospy.init_node("safety_monitor", anonymous=False)

        self.rate_hz = float(rospy.get_param("/safety_monitor/rate", 50.0))
        self.max_roll_deg = float(rospy.get_param("/safety_monitor/max_roll_deg", 12.0))
        self.max_pitch_deg = float(rospy.get_param("/safety_monitor/max_pitch_deg", 12.0))
        self.contact_loss_distance = float(rospy.get_param("/safety_monitor/contact_loss_distance", 0.08))
        self.distance_jump_threshold = float(rospy.get_param("/safety_monitor/distance_jump_threshold", 0.03))
        self.sensor_timeout = float(rospy.get_param("/safety_monitor/sensor_timeout", 0.5))
        self.mavros_timeout = float(rospy.get_param("/safety_monitor/mavros_timeout", 0.5))
        self.enable_sensor_timeout_check = bool(rospy.get_param("/safety_monitor/enable_sensor_timeout_check", False))
        self.enable_mavros_timeout_check = bool(rospy.get_param("/safety_monitor/enable_mavros_timeout_check", False))
        self.require_offboard = bool(rospy.get_param("/safety_monitor/require_offboard", True))
        self.require_armed = bool(rospy.get_param("/safety_monitor/require_armed", True))
        self.offboard_drop_emergency = bool(rospy.get_param("/safety_monitor/offboard_drop_emergency", True))
        self.enable_motor_output_warning = bool(
            rospy.get_param("/safety_monitor/enable_motor_output_warning", False)
        )
        self.motor_output_topic = str(
            rospy.get_param("/safety_monitor/motor_output_topic", "/mavros/actuator_control")
        )
        self.motor_output_warning_threshold = float(
            rospy.get_param("/safety_monitor/motor_output_warning_threshold", 0.85)
        )
        self.motor_output_indices = rospy.get_param("/safety_monitor/motor_output_indices", [])

        self.last_imu_time = rospy.Time(0)
        self.last_mavros_state_time = rospy.Time(0)
        self.last_distance_time = rospy.Time(0)
        self.last_pose_time = rospy.Time(0)
        self.last_motor_output_time = rospy.Time(0)

        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_distance = None
        self.prev_distance = None
        self.current_motor_outputs = []
        self.high_motor_outputs = []
        self.mavros_connected = False
        self.mavros_mode = ""
        self.mavros_armed = False
        self.current_phase = TaskPhase.IDLE

        self.state_pub = rospy.Publisher("/uav_contact/safety/state", SafetyState, queue_size=10)

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._on_pose, queue_size=10)
        rospy.Subscriber("/mavros/state", MavrosState, self._on_mavros_state, queue_size=10)
        rospy.Subscriber("/contact/distance", Float64, self._on_distance, queue_size=10)
        rospy.Subscriber("/uav_contact/task/phase", TaskPhase, self._on_task_phase, queue_size=10)
        if self.enable_motor_output_warning:
            rospy.Subscriber(self.motor_output_topic, ActuatorControl, self._on_motor_output, queue_size=10)

        rospy.loginfo("Safety monitor node started")

    def _on_mavros_state(self, msg):
        self.last_mavros_state_time = rospy.Time.now()
        self.mavros_connected = msg.connected
        self.mavros_mode = getattr(msg, "mode", "")
        self.mavros_armed = bool(getattr(msg, "armed", False))

    def _on_pose(self, msg):
        self.last_pose_time = rospy.Time.now()
        self.last_imu_time = self.last_pose_time
        q = msg.pose.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.current_roll = math.atan2(sinr_cosp, cosr_cosp) * 180.0 / math.pi

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            self.current_pitch = math.copysign(90.0, sinp)
        else:
            self.current_pitch = math.asin(sinp) * 180.0 / math.pi

    def _on_distance(self, msg):
        self.last_distance_time = rospy.Time.now()
        self.prev_distance = self.current_distance
        self.current_distance = msg.data

    def _on_task_phase(self, msg):
        self.current_phase = msg.phase

    def _on_motor_output(self, msg):
        self.last_motor_output_time = rospy.Time.now()
        self.current_motor_outputs = [float(value) for value in msg.controls]

        if self.motor_output_indices:
            candidate_indices = [int(index) for index in self.motor_output_indices]
        else:
            candidate_indices = list(range(len(self.current_motor_outputs)))

        high_outputs = []
        for index in candidate_indices:
            if index < 0 or index >= len(self.current_motor_outputs):
                continue
            normalized = abs(self.current_motor_outputs[index])
            if normalized > self.motor_output_warning_threshold:
                high_outputs.append((index, normalized))
        self.high_motor_outputs = high_outputs

    def _motor_output_warning_reason(self):
        if not self.high_motor_outputs:
            return ""
        details = " ".join(
            "motor{}={:.2f}".format(index, value)
            for index, value in self.high_motor_outputs
        )
        return "MOTOR_OUTPUT_HIGH {} threshold={:.2f}".format(
            details, self.motor_output_warning_threshold
        )

    def evaluate(self):
        now = rospy.Time.now()
        state = SafetyState.NORMAL
        safe = True
        require_emergency = False
        reason = "NORMAL"

        imu_age = (now - self.last_imu_time).to_sec()
        mavros_age = (now - self.last_mavros_state_time).to_sec()
        distance_age = (now - self.last_distance_time).to_sec()
        active_phases = (
            TaskPhase.APPROACH,
            TaskPhase.INITIAL_CONTACT,
            TaskPhase.SLIDING_CONTACT,
            TaskPhase.RETREAT,
        )

        if (
            (self.enable_mavros_timeout_check and mavros_age > self.mavros_timeout)
            or not self.mavros_connected
        ):
            state = SafetyState.MAVROS_DISCONNECTED
            safe = False
            reason = "MAVROS_DISCONNECTED"
            if self.current_phase in active_phases:
                require_emergency = True
        elif self.require_offboard and self.mavros_mode != "OFFBOARD":
            state = SafetyState.MAVROS_DISCONNECTED
            safe = False
            reason = "NOT_OFFBOARD mode={}".format(self.mavros_mode)
            if self.offboard_drop_emergency and self.current_phase in active_phases:
                require_emergency = True
        elif self.require_armed and not self.mavros_armed:
            state = SafetyState.MAVROS_DISCONNECTED
            safe = False
            reason = "NOT_ARMED"
            if self.offboard_drop_emergency and self.current_phase in active_phases:
                require_emergency = True
        elif self.enable_sensor_timeout_check and imu_age > self.sensor_timeout:
            state = SafetyState.SENSOR_TIMEOUT
            safe = False
            reason = "SENSOR_TIMEOUT"
        elif abs(self.current_roll) > self.max_roll_deg or abs(self.current_pitch) > self.max_pitch_deg:
            state = SafetyState.ATTITUDE_LIMIT_EXCEEDED
            safe = False
            require_emergency = True
            reason = "ATTITUDE_LIMIT_EXCEEDED roll={:.1f} pitch={:.1f}".format(
                self.current_roll, self.current_pitch)
        elif self.current_distance is not None and self.current_distance > self.contact_loss_distance:
            state = SafetyState.CONTACT_LOSS
            reason = "CONTACT_LOSS distance={:.3f}".format(self.current_distance)
            rospy.logwarn_throttle(1.0, reason)
        elif (
            self.current_distance is not None
            and self.prev_distance is not None
            and distance_age <= self.sensor_timeout
            and abs(self.current_distance - self.prev_distance) > self.distance_jump_threshold
        ):
            state = SafetyState.DISTANCE_JUMP
            safe = False
            reason = "DISTANCE_JUMP prev={:.3f} cur={:.3f}".format(
                self.prev_distance, self.current_distance)

        motor_warning = ""
        if self.enable_motor_output_warning:
            motor_warning = self._motor_output_warning_reason()
            if motor_warning:
                rospy.logwarn_throttle(1.0, motor_warning)
                if safe:
                    reason = motor_warning

        if require_emergency and self.current_phase in (
            TaskPhase.APPROACH, TaskPhase.INITIAL_CONTACT,
            TaskPhase.SLIDING_CONTACT
        ):
            state = SafetyState.EMERGENCY_RETREAT_REQUIRED

        return state, safe, require_emergency, reason

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            state_val, safe, require_emergency, reason = self.evaluate()

            msg = SafetyState()
            msg.header.stamp = rospy.Time.now()
            msg.state = state_val
            msg.safe = safe
            msg.require_emergency_retreat = require_emergency
            msg.reason = reason
            self.state_pub.publish(msg)

            rate.sleep()


def main():
    node = SafetyMonitorNode()
    node.spin()


if __name__ == "__main__":
    main()
