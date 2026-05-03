#!/usr/bin/env python3

import os
import signal
import subprocess
from datetime import datetime

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import State as MavrosState
from uav_contact_msgs.msg import TaskPhase, TrajectoryPoint


class ExperimentDataRecorderNode:
    def __init__(self):
        rospy.init_node("experiment_data_recorder", anonymous=False)

        self.output_dir = rospy.get_param("~output_dir", os.path.expanduser("~/rosbag/uav_contact"))
        self.prefix = rospy.get_param("~file_prefix", "uav_contact_exp")

        self.expected_pos = None
        self.expected_vel = None
        self.actual_pos = None
        self.actual_vel = None
        self.phase = TaskPhase.IDLE
        self.offboard_enabled = False

        self.pos_err_ref_minus_meas_pub = rospy.Publisher(
            "/uav_contact/diagnostics/position_error_ref_minus_meas", Vector3Stamped, queue_size=10
        )
        self.vel_err_ref_minus_meas_pub = rospy.Publisher(
            "/uav_contact/diagnostics/velocity_error_ref_minus_meas", Vector3Stamped, queue_size=10
        )

        rospy.Subscriber("/uav_contact/trajectory/reference", TrajectoryPoint, self._on_trajectory, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._on_pose, queue_size=10)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self._on_velocity, queue_size=10)
        rospy.Subscriber("/mavros/state", MavrosState, self._on_mavros_state, queue_size=10)
        rospy.Subscriber("/uav_contact/task/phase", TaskPhase, self._on_phase, queue_size=10)

        os.makedirs(self.output_dir, exist_ok=True)
        self.bag_process = self._start_rosbag_record()

        self.timer = rospy.Timer(rospy.Duration(1.0 / 50.0), self._publish_errors)
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo("Experiment data recorder started")

    def _on_trajectory(self, msg):
        self.expected_pos = (float(msg.x), float(msg.y), float(msg.z))
        self.expected_vel = (float(msg.vx), float(msg.vy), float(msg.vz))

    def _on_pose(self, msg):
        self.actual_pos = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )

    def _on_velocity(self, msg):
        self.actual_vel = (
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.linear.z),
        )

    def _on_mavros_state(self, msg):
        self.offboard_enabled = (msg.mode == "OFFBOARD")

    def _on_phase(self, msg):
        self.phase = msg.phase

    @staticmethod
    def _vector_msg(frame_id, xyz):
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.vector.x = float(xyz[0])
        msg.vector.y = float(xyz[1])
        msg.vector.z = float(xyz[2])
        return msg

    def _publish_errors(self, _event):
        if self.expected_pos is not None and self.actual_pos is not None:
            ref_minus_meas = (
                self.expected_pos[0] - self.actual_pos[0],
                self.expected_pos[1] - self.actual_pos[1],
                self.expected_pos[2] - self.actual_pos[2],
            )
            self.pos_err_ref_minus_meas_pub.publish(
                self._vector_msg("map", ref_minus_meas)
            )

        if self.expected_vel is not None and self.actual_vel is not None:
            ref_minus_meas = (
                self.expected_vel[0] - self.actual_vel[0],
                self.expected_vel[1] - self.actual_vel[1],
                self.expected_vel[2] - self.actual_vel[2],
            )
            self.vel_err_ref_minus_meas_pub.publish(
                self._vector_msg("map", ref_minus_meas)
            )

    def _start_rosbag_record(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_path = os.path.join(self.output_dir, "{}_{}.bag".format(self.prefix, ts))

        topics = [
            "/uav_contact/trajectory/reference",
            "/mavros/local_position/pose",
            "/mavros/local_position/velocity_local",
            "/uav_contact/diagnostics/position_error_ref_minus_meas",
            "/uav_contact/diagnostics/velocity_error_ref_minus_meas",
            "/mavros/state",
            "/uav_contact/task/phase",
            "/uav_contact/safety/state",
            "/uav_contact/contact/normal_velocity_cmd",
            "/mavros/setpoint_raw/local",
        ]

        cmd = ["rosbag", "record", "-O", bag_path] + topics
        rospy.loginfo("Starting rosbag: %s", " ".join(cmd))
        return subprocess.Popen(cmd, preexec_fn=os.setsid)

    def _shutdown(self):
        if self.bag_process is None:
            return
        if self.bag_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
                self.bag_process.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.bag_process.pid), signal.SIGTERM)
                except Exception:
                    pass
        rospy.loginfo("Experiment data recorder stopped")


if __name__ == "__main__":
    node = ExperimentDataRecorderNode()
    rospy.spin()
