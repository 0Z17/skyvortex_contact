#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import numpy as np

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool, Float64
    from uav_contact_msgs.msg import TaskPhase, TrajectoryPoint
except ImportError:  # pragma: no cover
    class _Obj:
        pass

    class _FakeTime:
        @staticmethod
        def now():
            return 0.0

    rospy = _Obj()
    rospy.Time = _FakeTime
    rospy.loginfo = lambda *args, **kwargs: None
    rospy.logwarn = lambda *args, **kwargs: None

    class Float64:
        def __init__(self, data=0.0):
            self.data = float(data)

    class TaskPhase:
        IDLE = 0
        STABILIZE = 1
        APPROACH = 2
        INITIAL_CONTACT = 3
        SLIDING_CONTACT = 4
        RETREAT = 5
        EMERGENCY_RETREAT = 6
        FINISHED = 7
        ERROR = 8

        def __init__(self):
            self.phase = TaskPhase.IDLE

    class TrajectoryPoint:
        def __init__(self):
            self.header = _Obj()
            self.header.stamp = None
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


class TrajectoryServer:
    def __init__(
        self,
        trajectory_publisher=None,
        joint_publisher=None,
        publish_rate_hz=50.0,
        leave_time_sec=20.0,
        approach_offset_m=0.5,
        approach_time_sec=30.0,
        initial_contact_time_sec=5.0,
        retreat_distance_m=0.5,
        semi_auto_mode=False,
        manual_axis1_max_velocity=0.2,
        manual_axis2_max_velocity=0.2,
        manual_default_theta=0.0,
    ):
        self.trajectory_publisher = trajectory_publisher
        self.joint_publisher = joint_publisher
        self.sliding_done_publisher = None
        self.publish_rate_hz = float(publish_rate_hz)
        self.leave_time_sec = float(leave_time_sec)
        self.approach_offset_m = float(approach_offset_m)
        self.approach_time_sec = float(approach_time_sec)
        self.initial_contact_time_sec = float(initial_contact_time_sec)
        self.retreat_distance_m = float(retreat_distance_m)
        self.semi_auto_mode = bool(semi_auto_mode)
        self.manual_axis1_max_velocity = float(manual_axis1_max_velocity)
        self.manual_axis2_max_velocity = float(manual_axis2_max_velocity)
        self.manual_default_theta = float(manual_default_theta)
        self.manual_axis1 = 0.0
        self.manual_axis2 = 0.0
        self.current_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "psi": 0.0}
        self.current_theta = self.manual_default_theta
        self.min_approach_speed_mps = 0.15
        self.waypoints = []
        self.waypoint_index = 0
        self.phase = TaskPhase.IDLE
        self._prev_phase = TaskPhase.IDLE
        self._last_waypoint = None
        self._stabilize_start_waypoint = None

        self.approach_path = []
        self.approach_index = 0
        self.approach_active = False

        self.initial_contact_path = []
        self.initial_contact_index = 0
        self.initial_contact_active = False

        self.cost_res = 0.0005
        self.sliding_path = []
        self.sliding_index = 0
        self.sliding_active = False
        self._last_sliding_log_time = None

        self.retreat_path = []
        self.retreat_index = 0
        self.retreat_active = False

    def load_csv(self, csv_path):
        path = Path(csv_path)
        if not path.is_file():
            raise FileNotFoundError("CSV file not found: {}".format(csv_path))

        waypoints = []
        with path.open("r", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            required_columns = {"X", "Y", "Z", "Psi", "Theta"}
            fieldnames = set(reader.fieldnames or [])
            missing_columns = sorted(required_columns - fieldnames)
            if missing_columns:
                raise ValueError(
                    "Missing required CSV columns: {}".format(", ".join(missing_columns))
                )

            previous_row = None
            for row_index, row in enumerate(reader, start=2):
                try:
                    waypoint = {
                        "x": float(row["X"]),
                        "y": float(row["Y"]),
                        "z": float(row["Z"]),
                        "psi": float(row["Psi"]),
                        "theta": float(row["Theta"]),
                    }
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        "Non-numeric value in CSV at row {}: {}".format(row_index, row)
                    )

                if previous_row is None:
                    waypoint["vx"] = 0.0
                    waypoint["vy"] = 0.0
                    waypoint["vz"] = 0.0
                    waypoint["vpsi"] = 0.0
                    waypoint["vtheta"] = 0.0
                else:
                    waypoint["vx"] = waypoint["x"] - previous_row["x"]
                    waypoint["vy"] = waypoint["y"] - previous_row["y"]
                    waypoint["vz"] = waypoint["z"] - previous_row["z"]
                    waypoint["vpsi"] = waypoint["psi"] - previous_row["psi"]
                    waypoint["vtheta"] = waypoint["theta"] - previous_row["theta"]

                waypoint["nx"] = math.cos(waypoint["psi"]) * math.cos(waypoint["theta"])
                waypoint["ny"] = math.sin(waypoint["psi"]) * math.cos(waypoint["theta"])
                waypoint["nz"] = math.sin(waypoint["theta"])

                waypoints.append(waypoint)
                previous_row = waypoint

        return waypoints

    def _normal_from_angles(self, psi, theta):
        return (
            math.cos(psi) * math.cos(theta),
            math.sin(psi) * math.cos(theta),
            math.sin(theta),
        )

    def _make_waypoint(self, x, y, z, psi, theta):
        wp = {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "psi": float(psi),
            "theta": float(theta),
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "vpsi": 0.0,
            "vtheta": 0.0,
        }
        wp["nx"], wp["ny"], wp["nz"] = self._normal_from_angles(wp["psi"], wp["theta"])
        return wp

    def update_local_pose(self, x, y, z, psi=None, theta=0.0):
        if psi is None:
            if self._last_waypoint is not None:
                psi = float(self._last_waypoint["psi"])
            elif self.waypoints:
                psi = float(self.waypoints[0]["psi"])
            else:
                psi = 0.0
        self.current_pose = {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "psi": float(psi),
        }
        wp = self._make_waypoint(x=float(x), y=float(y), z=float(z), psi=float(psi), theta=float(theta))
        self._stabilize_start_waypoint = wp
        if self.phase == TaskPhase.STABILIZE:
            self._last_waypoint = wp

    def update_manual_axis1(self, value):
        self.manual_axis1 = max(min(float(value), 1.0), -1.0)

    def update_manual_axis2(self, value):
        self.manual_axis2 = max(min(float(value), 1.0), -1.0)

    def update_joint_theta(self, value):
        self.current_theta = float(value)

    @staticmethod
    def manual_tangent_basis(yaw, theta):
        normal = np.array([
            math.cos(yaw) * math.cos(theta),
            math.sin(yaw) * math.cos(theta),
            math.sin(theta),
        ], dtype=float)
        tangent_1 = np.array([-math.sin(yaw), math.cos(yaw), 0.0], dtype=float)
        tangent_2 = np.cross(normal, tangent_1)

        normal = normal / max(np.linalg.norm(normal), 1e-9)
        tangent_1 = tangent_1 / max(np.linalg.norm(tangent_1), 1e-9)
        tangent_2 = tangent_2 / max(np.linalg.norm(tangent_2), 1e-9)
        return normal, tangent_1, tangent_2

    def _publish_manual_reference(self):
        pose = self.current_pose
        theta = self.current_theta
        normal, tangent_1, tangent_2 = self.manual_tangent_basis(pose["psi"], theta)
        velocity = (
            self.manual_axis1 * self.manual_axis1_max_velocity * tangent_1
            + self.manual_axis2 * self.manual_axis2_max_velocity * tangent_2
        )

        msg = TrajectoryPoint()
        msg.header.stamp = rospy.Time.now()
        msg.x = pose["x"]
        msg.y = pose["y"]
        msg.z = pose["z"]
        msg.psi = pose["psi"]
        msg.theta = theta
        msg.vx = float(velocity[0])
        msg.vy = float(velocity[1])
        msg.vz = float(velocity[2])
        msg.vpsi = 0.0
        msg.vtheta = 0.0
        msg.nx = float(normal[0])
        msg.ny = float(normal[1])
        msg.nz = float(normal[2])
        self.trajectory_publisher.publish(msg)
        if self.joint_publisher:
            self.joint_publisher.publish(Float64(data=theta))
        self._last_waypoint = {
            "x": msg.x, "y": msg.y, "z": msg.z, "psi": msg.psi, "theta": msg.theta,
            "vx": msg.vx, "vy": msg.vy, "vz": msg.vz, "vpsi": 0.0, "vtheta": 0.0,
            "nx": msg.nx, "ny": msg.ny, "nz": msg.nz,
        }

    @staticmethod
    def _normalized(vx, vy, vz):
        norm = math.sqrt(vx * vx + vy * vy + vz * vz)
        if norm <= 1e-9:
            return 1.0, 0.0, 0.0
        return vx / norm, vy / norm, vz / norm

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _build_segment(self, start_wp, end_wp, duration_sec):
        num = max(2, int(float(duration_sec) * self.publish_rate_hz))
        step_x = (float(end_wp["x"]) - float(start_wp["x"])) / float(num)
        step_y = (float(end_wp["y"]) - float(start_wp["y"])) / float(num)
        step_z = (float(end_wp["z"]) - float(start_wp["z"])) / float(num)
        step_psi = (float(end_wp["psi"]) - float(start_wp["psi"])) / float(num)
        step_theta = (float(end_wp["theta"]) - float(start_wp["theta"])) / float(num)
        segment = []
        for idx in range(num):
            alpha = idx / float(num - 1)
            psi = (1.0 - alpha) * float(start_wp["psi"]) + alpha * float(end_wp["psi"])
            theta = (1.0 - alpha) * float(start_wp["theta"]) + alpha * float(end_wp["theta"])
            wp = self._make_waypoint(
                x=(1.0 - alpha) * float(start_wp["x"]) + alpha * float(end_wp["x"]),
                y=(1.0 - alpha) * float(start_wp["y"]) + alpha * float(end_wp["y"]),
                z=(1.0 - alpha) * float(start_wp["z"]) + alpha * float(end_wp["z"]),
                psi=psi,
                theta=theta,
            )
            wp["vx"] = step_x
            wp["vy"] = step_y
            wp["vz"] = step_z
            wp["vpsi"] = step_psi
            wp["vtheta"] = step_theta
            segment.append(wp)
        return segment

    def _segment_cost(self, start_wp, end_wp):
        weights = np.array([1.0, 1.0, 1.0, 1.0, 3.0], dtype=float)
        start = np.array([
            float(start_wp["x"]),
            float(start_wp["y"]),
            float(start_wp["z"]),
            float(start_wp["psi"]),
            float(start_wp["theta"]),
        ], dtype=float)
        end = np.array([
            float(end_wp["x"]),
            float(end_wp["y"]),
            float(end_wp["z"]),
            float(end_wp["psi"]),
            float(end_wp["theta"]),
        ], dtype=float)
        return float(np.linalg.norm(weights * (end - start)))

    def _build_sliding_path(self):
        if len(self.waypoints) < 2:
            self.sliding_path = list(self.waypoints)
            self.sliding_index = 0
            self.sliding_active = bool(self.sliding_path)
            return

        sliding_path = []
        for idx in range(1, len(self.waypoints)):
            start_wp = self.waypoints[idx - 1]
            end_wp = self.waypoints[idx]
            cost = self._segment_cost(start_wp, end_wp)
            num = max(2, int(math.ceil(cost / self.cost_res)))
            interpolate = np.linspace(
                np.array([start_wp["x"], start_wp["y"], start_wp["z"], start_wp["psi"], start_wp["theta"]], dtype=float),
                np.array([end_wp["x"], end_wp["y"], end_wp["z"], end_wp["psi"], end_wp["theta"]], dtype=float),
                num=num,
            )
            step = (interpolate[1] - interpolate[0]) if len(interpolate) > 1 else np.zeros(5, dtype=float)
            for row in interpolate[:-1]:
                wp = self._make_waypoint(row[0], row[1], row[2], row[3], row[4])
                wp["vx"] = float(step[0])
                wp["vy"] = float(step[1])
                wp["vz"] = float(step[2])
                wp["vpsi"] = float(step[3])
                wp["vtheta"] = float(step[4])
                sliding_path.append(wp)
        last_wp = self._make_waypoint(
            self.waypoints[-1]["x"], self.waypoints[-1]["y"], self.waypoints[-1]["z"],
            self.waypoints[-1]["psi"], self.waypoints[-1]["theta"]
        )
        if sliding_path:
            last_wp["vx"] = float(sliding_path[-1]["vx"])
            last_wp["vy"] = float(sliding_path[-1]["vy"])
            last_wp["vz"] = float(sliding_path[-1]["vz"])
            last_wp["vpsi"] = float(sliding_path[-1]["vpsi"])
            last_wp["vtheta"] = float(sliding_path[-1]["vtheta"])
        sliding_path.append(last_wp)
        self.sliding_path = sliding_path
        self.sliding_index = 0
        self.sliding_active = bool(sliding_path)

    def _log_sliding_progress(self):
        if not self.sliding_path:
            return
        now = rospy.Time.now()
        if self._last_sliding_log_time is not None:
            delta = now - self._last_sliding_log_time
            elapsed_sec = delta.to_sec() if hasattr(delta, "to_sec") else float(delta)
            if elapsed_sec < 1.0:
                return
        total = len(self.sliding_path)
        current = min(self.sliding_index + 1, total)
        ratio = (100.0 * current / float(total)) if total > 0 else 100.0
        rospy.loginfo(
            "SLIDING progress: %d/%d (%.1f%%)",
            current,
            total,
            ratio,
        )
        self._last_sliding_log_time = now
    def _build_approach_path(self):
        if not self.waypoints:
            self.approach_path = []
            self.approach_index = 0
            self.approach_active = False
            return

        wp0 = self.waypoints[0]
        nx, ny, nz = self._normalized(float(wp0["nx"]), float(wp0["ny"]), float(wp0["nz"]))

        if self._stabilize_start_waypoint is not None:
            stable_wp = self._stabilize_start_waypoint
        elif self._last_waypoint is not None:
            stable_wp = self._last_waypoint
        else:
            stable_wp = self._make_waypoint(0.0, 0.0, 0.0, float(wp0["psi"]), 0.0)

        approach_end = self._make_waypoint(
            x=float(wp0["x"]) - self.approach_offset_m * nx,
            y=float(wp0["y"]) - self.approach_offset_m * ny,
            z=float(wp0["z"]) - self.approach_offset_m * nz,
            psi=float(wp0["psi"]),
            theta=0.0,
        )

        approach_distance = math.sqrt(
            (float(approach_end["x"]) - float(stable_wp["x"])) ** 2
            + (float(approach_end["y"]) - float(stable_wp["y"])) ** 2
            + (float(approach_end["z"]) - float(stable_wp["z"])) ** 2
        )
        required_duration = approach_distance / max(self.min_approach_speed_mps, 1e-6)
        if required_duration > self.approach_time_sec + 1e-6:
            rospy.logwarn(
                "APPROACH time may be insufficient: configured=%.2fs required>=%.2fs distance=%.3fm",
                self.approach_time_sec,
                required_duration,
                approach_distance,
            )

        self.approach_path = self._build_segment(
            start_wp=stable_wp,
            end_wp=approach_end,
            duration_sec=self.approach_time_sec,
        )
        self.approach_index = 0
        self.approach_active = bool(self.approach_path)

    def _build_initial_contact_path(self):
        if not self.waypoints:
            self.initial_contact_path = []
            self.initial_contact_index = 0
            self.initial_contact_active = False
            return

        wp0 = self.waypoints[0]
        nx, ny, nz = self._normalized(float(wp0["nx"]), float(wp0["ny"]), float(wp0["nz"]))

        approach_end = self._make_waypoint(
            x=float(wp0["x"]) - self.approach_offset_m * nx,
            y=float(wp0["y"]) - self.approach_offset_m * ny,
            z=float(wp0["z"]) - self.approach_offset_m * nz,
            psi=float(wp0["psi"]),
            theta=0.0,
        )
        wp0_contact = self._make_waypoint(
            x=float(wp0["x"]),
            y=float(wp0["y"]),
            z=float(wp0["z"]),
            psi=float(wp0["psi"]),
            theta=float(wp0["theta"]),
        )

        self.initial_contact_path = self._build_segment(
            start_wp=approach_end,
            end_wp=wp0_contact,
            duration_sec=self.initial_contact_time_sec,
        )
        self.initial_contact_index = 0
        self.initial_contact_active = bool(self.initial_contact_path)

    def _build_retreat_path(self):
        if self._last_waypoint is not None:
            start_wp = self._last_waypoint
        elif self.waypoints:
            start_wp = self.waypoints[-1]
        else:
            start_wp = {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "psi": 0.0,
                "theta": 0.0,
                "nx": 1.0,
                "ny": 0.0,
                "nz": 0.0,
            }

        nx, ny, nz = self._normalized(
            float(start_wp.get("nx", 1.0)),
            float(start_wp.get("ny", 0.0)),
            float(start_wp.get("nz", 0.0)),
        )

        end_wp = self._make_waypoint(
            x=float(start_wp["x"]) - self.retreat_distance_m * nx,
            y=float(start_wp["y"]) - self.retreat_distance_m * ny,
            z=float(start_wp["z"]) - self.retreat_distance_m * nz,
            psi=float(start_wp["psi"]),
            theta=0.0,
        )

        self.retreat_path = self._build_segment(
            start_wp=start_wp,
            end_wp=end_wp,
            duration_sec=self.leave_time_sec,
        )
        for wp in self.retreat_path:
            wp["nx"] = nx
            wp["ny"] = ny
            wp["nz"] = nz
        self.retreat_index = 0
        self.retreat_active = bool(self.retreat_path)

    def set_phase(self, phase):
        prev_phase = self.phase
        self.phase = phase
        if self.phase == TaskPhase.APPROACH and prev_phase != TaskPhase.APPROACH:
            self._build_approach_path()
        if self.phase == TaskPhase.INITIAL_CONTACT and prev_phase != TaskPhase.INITIAL_CONTACT:
            self._build_initial_contact_path()
        if self.phase == TaskPhase.SLIDING_CONTACT and prev_phase != TaskPhase.SLIDING_CONTACT:
            self._build_sliding_path()
            self._last_sliding_log_time = None
        if self.phase == TaskPhase.RETREAT and prev_phase != TaskPhase.RETREAT:
            self._build_retreat_path()
        self._prev_phase = prev_phase

    def _publish_hover(self):
        if self._last_waypoint is None:
            msg = TrajectoryPoint()
            msg.header.stamp = rospy.Time.now()
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0
            msg.psi = 0.0
            msg.theta = 0.0
            msg.vx = 0.0
            msg.vy = 0.0
            msg.vz = 0.0
            msg.vpsi = 0.0
            msg.vtheta = 0.0
            msg.nx = 1.0
            msg.ny = 0.0
            msg.nz = 0.0
            self.trajectory_publisher.publish(msg)
            if self.joint_publisher:
                self.joint_publisher.publish(Float64(data=0.0))
            return

        wp = self._last_waypoint
        msg = TrajectoryPoint()
        msg.header.stamp = rospy.Time.now()
        msg.x = float(wp["x"])
        msg.y = float(wp["y"])
        msg.z = float(wp["z"])
        msg.psi = float(wp["psi"])
        msg.theta = float(wp["theta"])
        msg.vx = 0.0
        msg.vy = 0.0
        msg.vz = 0.0
        msg.vpsi = 0.0
        msg.vtheta = 0.0
        msg.nx = float(wp["nx"])
        msg.ny = float(wp["ny"])
        msg.nz = float(wp["nz"])
        self.trajectory_publisher.publish(msg)
        if self.joint_publisher:
            self.joint_publisher.publish(Float64(data=float(wp["theta"])))

    def _publish_waypoint(self, wp, zero_velocity=False):
        msg = TrajectoryPoint()
        msg.header.stamp = rospy.Time.now()
        msg.x = float(wp["x"])
        msg.y = float(wp["y"])
        msg.z = float(wp["z"])
        msg.psi = float(wp["psi"])
        msg.theta = float(wp["theta"])
        if zero_velocity:
            msg.vx = 0.0
            msg.vy = 0.0
            msg.vz = 0.0
            msg.vpsi = 0.0
            msg.vtheta = 0.0
        else:
            msg.vx = float(wp["vx"]) * self.publish_rate_hz
            msg.vy = float(wp["vy"]) * self.publish_rate_hz
            msg.vz = float(wp["vz"]) * self.publish_rate_hz
            msg.vpsi = float(wp["vpsi"]) * self.publish_rate_hz
            msg.vtheta = float(wp["vtheta"]) * self.publish_rate_hz
        msg.nx = float(wp["nx"])
        msg.ny = float(wp["ny"])
        msg.nz = float(wp["nz"])
        self.trajectory_publisher.publish(msg)
        if self.joint_publisher:
            self.joint_publisher.publish(Float64(data=float(wp["theta"])))

        self._last_waypoint = wp

    def _advance_waypoint(self):
        if not self.waypoints:
            return
        if self.waypoint_index < len(self.waypoints) - 1:
            self.waypoint_index += 1

    def publish(self):
        if self.semi_auto_mode:
            if self.phase in (TaskPhase.STABILIZE, TaskPhase.SLIDING_CONTACT):
                self._publish_manual_reference()
            return

        if not self.waypoints:
            return

        if self.phase == TaskPhase.IDLE or self.phase == TaskPhase.FINISHED:
            return

        if self.phase == TaskPhase.STABILIZE:
            self._publish_hover()
            return

        if self.phase == TaskPhase.EMERGENCY_RETREAT or self.phase == TaskPhase.ERROR:
            self._publish_hover()
            return

        if self.phase == TaskPhase.APPROACH:
            if self.approach_active and self.approach_index < len(self.approach_path):
                wp = self.approach_path[self.approach_index]
                self._publish_waypoint(wp)
                if self.approach_index < len(self.approach_path) - 1:
                    self.approach_index += 1
                return
            self._publish_hover()
            return

        if self.phase == TaskPhase.INITIAL_CONTACT:
            if self.initial_contact_active and self.initial_contact_index < len(self.initial_contact_path):
                wp = self.initial_contact_path[self.initial_contact_index]
                self._publish_waypoint(wp)
                if self.initial_contact_index < len(self.initial_contact_path) - 1:
                    self.initial_contact_index += 1
                return
            self._publish_hover()
            return

        if self.phase == TaskPhase.SLIDING_CONTACT:
            if self.sliding_active and self.sliding_index < len(self.sliding_path):
                self._log_sliding_progress()
                wp = self.sliding_path[self.sliding_index]
                self._publish_waypoint(wp)
                if self.sliding_index <= len(self.sliding_path) - 1:
                    self.sliding_index += 1
                return
            if (self.sliding_done_publisher is not None) and (self.sliding_active):
                self.sliding_done_publisher.publish(Bool(data=True))
            self._publish_hover()
            return

        if self.phase == TaskPhase.RETREAT:
            if self.retreat_active and self.retreat_index < len(self.retreat_path):
                wp = self.retreat_path[self.retreat_index]
                self._publish_waypoint(wp)
                if self.retreat_index < len(self.retreat_path) - 1:
                    self.retreat_index += 1
                return
            self._publish_hover()
            return


def main():
    rospy.init_node("trajectory_server", anonymous=False)

    path_csv = rospy.get_param("/trajectory_server/path_csv", "/home/wsl/skyvortex_contact/catkin_ws/src/uav_contact_core/scripts/trajectory/trajectory_server_node.py")
    publish_rate_hz = float(rospy.get_param("/trajectory_server/publish_rate_hz", 50.0))
    leave_time_sec = float(rospy.get_param("/trajectory_server/leave_time_sec", 20.0))
    approach_offset_m = float(rospy.get_param("/trajectory_server/approach_offset_m", 0.3))
    approach_time_sec = float(rospy.get_param("/trajectory_server/approach_time_sec", 30.0))
    initial_contact_time_sec = float(rospy.get_param("/trajectory_server/initial_contact_time_sec", 5.0))
    retreat_distance_m = float(rospy.get_param("/trajectory_server/retreat_distance_m", 0.5))
    semi_auto_mode = bool(rospy.get_param("/trajectory_server/semi_auto_mode", False))
    manual_axis1_topic = rospy.get_param("/rc_manager/tangent_axis1_topic", "/uav_contact/rc/tangent_axis1")
    manual_axis2_topic = rospy.get_param("/rc_manager/tangent_axis2_topic", "/uav_contact/rc/tangent_axis2")
    joint_state_topic = rospy.get_param("/topics/joint_state", "/uav_contact/joint/state")

    resolved_csv_path = path_csv
    prefix = "$(find uav_contact_core)"
    if path_csv.startswith(prefix):
        package_path = Path(__file__).resolve().parents[2]
        resolved_csv_path = str(package_path / path_csv[len(prefix):].lstrip("/"))

    trajectory_publisher = rospy.Publisher(
        "/uav_contact/trajectory/reference", TrajectoryPoint, queue_size=10
    )
    joint_publisher = rospy.Publisher(
        "/uav_contact/joint/reference", Float64, queue_size=10
    )

    sliding_done_publisher = rospy.Publisher(
        "/uav_contact/task/sliding_done", Bool, queue_size=1, latch=True
    )

    server = TrajectoryServer(
        trajectory_publisher=trajectory_publisher,
        joint_publisher=joint_publisher,
        publish_rate_hz=publish_rate_hz,
        leave_time_sec=leave_time_sec,
        approach_offset_m=approach_offset_m,
        approach_time_sec=approach_time_sec,
        initial_contact_time_sec=initial_contact_time_sec,
        retreat_distance_m=retreat_distance_m,
        semi_auto_mode=semi_auto_mode,
        manual_axis1_max_velocity=float(rospy.get_param("/trajectory_server/manual_axis1_max_velocity", 0.2)),
        manual_axis2_max_velocity=float(rospy.get_param("/trajectory_server/manual_axis2_max_velocity", 0.2)),
        manual_default_theta=float(rospy.get_param("/trajectory_server/manual_default_theta", 0.0)),
    )
    server.sliding_done_publisher = sliding_done_publisher

    if not semi_auto_mode:
        try:
            waypoints = server.load_csv(resolved_csv_path)
            server.waypoints = waypoints
        except (FileNotFoundError, ValueError) as exc:
            rospy.logerr("Failed to load trajectory CSV: {}".format(exc))
            return
    else:
        rospy.loginfo("Trajectory server running in semi-auto RC mode; CSV loading skipped")

    def _on_task_phase(msg):
        server.set_phase(msg.phase)

    def _on_local_pose(msg):
        current_yaw = server._yaw_from_quaternion(msg.pose.orientation)
        server.update_local_pose(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z,
            psi=current_yaw,
        )

    rospy.Subscriber("/uav_contact/task/phase", TaskPhase, _on_task_phase, queue_size=10)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, _on_local_pose, queue_size=10)
    if semi_auto_mode:
        rospy.Subscriber(manual_axis1_topic, Float64, lambda msg: server.update_manual_axis1(msg.data), queue_size=10)
        rospy.Subscriber(manual_axis2_topic, Float64, lambda msg: server.update_manual_axis2(msg.data), queue_size=10)
        rospy.Subscriber(joint_state_topic, Float64, lambda msg: server.update_joint_theta(msg.data), queue_size=10)

    rate = rospy.Rate(publish_rate_hz)
    rospy.loginfo("Trajectory server started at {} Hz".format(publish_rate_hz))

    while not rospy.is_shutdown():
        server.publish()
        rate.sleep()


if __name__ == "__main__":
    main()
