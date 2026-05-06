#!/usr/bin/env python3

try:
    import rospy
    from std_msgs.msg import Float64
    from uav_contact_msgs.msg import TaskPhase
except ImportError:  # pragma: no cover
    rospy = None

    class Float64:
        def __init__(self, data=0.0):
            self.data = float(data)

    class TaskPhase:
        APPROACH = 2
        INITIAL_CONTACT = 3
        SLIDING_CONTACT = 4
        RETREAT = 5


def clamp_joint_angle(angle, joint_min, joint_max):
    return max(min(float(angle), float(joint_max)), float(joint_min))


def bridge_param(name, default):
    return rospy.get_param(
        "/joint_servo_bridge/{}".format(name),
        rospy.get_param("~{}".format(name), default),
    )


class JointServoBridge:
    def __init__(self, joint_min=-1.57, joint_max=1.57, neutral_theta=0.0):
        self.joint_min = float(joint_min)
        self.joint_max = float(joint_max)
        if self.joint_max <= self.joint_min:
            raise ValueError("joint_max must be > joint_min")
        self.neutral_theta = float(neutral_theta)
        self.command_theta = 0.0
        self.measured_theta = 0.0
        self.phase_enabled = False

    def update_theta(self, theta):
        self.command_theta = clamp_joint_angle(theta, self.joint_min, self.joint_max)

    def current_joint_command(self):
        if not self.phase_enabled:
            return self.neutral_theta
        return self.command_theta

    def set_phase_enabled(self, enabled):
        self.phase_enabled = bool(enabled)


def main():
    if rospy is None:
        raise RuntimeError("rospy, std_msgs and uav_contact_msgs are required to run joint_servo_bridge.py")

    rospy.init_node("joint_servo_bridge", anonymous=False)
    publish_rate_hz = float(bridge_param("publish_rate_hz", 50.0))

    bridge = JointServoBridge(
        joint_min=float(bridge_param("joint_min", -1.57)),
        joint_max=float(bridge_param("joint_max", 1.57)),
        neutral_theta=float(bridge_param("neutral_theta", 0.0)),
    )

    enabled_phases = bridge_param(
        "enable_in_phases",
        [TaskPhase.APPROACH, TaskPhase.INITIAL_CONTACT,
         TaskPhase.SLIDING_CONTACT, TaskPhase.RETREAT],
    )

    output_mode = str(bridge_param("output_mode", "dynamixel")).strip().lower()
    if output_mode not in ("dynamixel", "topic"):
        rospy.logwarn("Unknown output_mode '%s', fallback to dynamixel", output_mode)
        output_mode = "dynamixel"
    servo_joint_offset = float(bridge_param("servo_joint_offset", 0.0))
    servo_profile_velocity = int(bridge_param("servo_profile_velocity", 30))
    servo_operating_mode = int(bridge_param("servo_operating_mode", 4))

    ctrl = None
    if output_mode == "dynamixel":
        try:
            import os
            import sys

            sys.path.insert(0, os.path.dirname(__file__))
            from dynamixel_control import DynamixelController

            ctrl = DynamixelController(
                dxl_id=int(bridge_param("dxl_id", 1)),
                baudrate=int(bridge_param("dxl_baudrate", 57600)),
                devicename=str(bridge_param("dxl_devicename", "/dev/uav/joint_servo")),
            )
            ctrl.initialize()
            ctrl.set_operating_mode(servo_operating_mode)
            ctrl.set_profile_vel(servo_profile_velocity)
            rospy.loginfo("Dynamixel controller connected")
        except Exception as exc:
            rospy.logwarn("Dynamixel init failed, fallback to topic-only mode: %s", exc)
            ctrl = None

    def _task_phase_callback(msg):
        bridge.set_phase_enabled(msg.phase in enabled_phases)

    def _joint_ref_callback(msg):
        bridge.update_theta(msg.data)

    rospy.Subscriber("/uav_contact/task/phase", TaskPhase, _task_phase_callback, queue_size=10)
    rospy.Subscriber("/uav_contact/joint/reference", Float64, _joint_ref_callback, queue_size=10)

    servo_cmd_pub = rospy.Publisher("/servo/command", Float64, queue_size=10)
    joint_state_pub = rospy.Publisher("/uav_contact/joint/state", Float64, queue_size=10)

    rate = rospy.Rate(publish_rate_hz)
    if ctrl is not None:
        rospy.loginfo("Joint servo bridge started in dynamixel mode")
    else:
        rospy.loginfo("Joint servo bridge started in topic-only mode")

    while not rospy.is_shutdown():
        joint_cmd = bridge.current_joint_command()
        servo_cmd_pub.publish(Float64(data=float(joint_cmd)))

        if ctrl is not None:
            try:
                ctrl.set_pos_rad(float(joint_cmd) + servo_joint_offset)
                measured = float(ctrl.get_present_rad()) - servo_joint_offset
                bridge.measured_theta = clamp_joint_angle(measured, bridge.joint_min, bridge.joint_max)
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "Dynamixel runtime error: %s", exc)

        joint_state_pub.publish(Float64(data=bridge.measured_theta))
        rate.sleep()


if __name__ == "__main__":
    main()
