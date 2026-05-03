#!/usr/bin/env python3

import csv
import math
from pathlib import Path

try:
    import rospy
    from std_msgs.msg import Float64
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
        publish_rate_hz=20.0,
        leave_time_sec=20.0,
        approach_offset_m=0.3,
        approach_time_sec=30.0,
        initial_contact_time_sec=5.0,
    ):
        self.trajectory_publisher = trajectory_publisher
        self.joint_publisher = joint_publisher
        self.publish_rate_hz = float(publish_rate_hz)
        self.leave_time_sec = float(leave_time_sec)
        self.approach_offset_m = float(approach_offset_m)
        self.approach_time_sec = float(approach_time_sec)
        self.initial_contact_time_sec = float(initial_contact_time_sec)
        self.waypoints = []
        self.waypoint_index = 0
        self.phase = TaskPhase.IDLE
        self._prev_phase = TaskPhase.IDLE
        self._last_waypoint = None

        self.approach_path = []
        self.approach_index = 0
        self.approach_active = False

        self.initial_contact_path = []
        self.initial_contact_index = 0
        self.initial_contact_active = False

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
            required_columns = {"x", "y", "z", "psi", "theta"}
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
                        "x": float(row["x"]),
                        "y": float(row["y"]),
                        "z": float(row["z"]),
                        "psi": float(row["psi"]),
                        "theta": float(row["theta"]),
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

    @staticmethod
    def _normalized(vx, vy, vz):
        norm = math.sqrt(vx * vx + vy * vy + vz * vz)
        if norm <= 1e-9:
            return 1.0, 0.0, 0.0
        return vx / norm, vy / norm, vz / norm

    def _build_segment(self, start_wp, end_wp, duration_sec):
        num = max(2, int(float(duration_sec) * self.publish_rate_hz))
        segment = []
        for idx in range(num):
            alpha = idx / float(num - 1)
            psi = (1.0 - alpha) * float(start_wp["psi"]) + alpha * float(end_wp["psi"])
            theta = (1.0 - alpha) * float(start_wp["theta"]) + alpha * float(end_wp["theta"])
            segment.append(
                self._make_waypoint(
                    x=(1.0 - alpha) * float(start_wp["x"]) + alpha * float(end_wp["x"]),
                    y=(1.0 - alpha) * float(start_wp["y"]) + alpha * float(end_wp["y"]),
                    z=(1.0 - alpha) * float(start_wp["z"]) + alpha * float(end_wp["z"]),
                    psi=psi,
                    theta=theta,
                )
            )
        return segment

    def _build_approach_path(self):
        if not self.waypoints:
            self.approach_path = []
            self.approach_index = 0
            self.approach_active = False
            return

        wp0 = self.waypoints[0]
        nx, ny, nz = self._normalized(float(wp0["nx"]), float(wp0["ny"]), float(wp0["nz"]))

        if self._last_waypoint is not None:
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

        end_wp = self._make_waypoint(
            x=float(start_wp["x"]) - 0.5,
            y=float(start_wp["y"]),
            z=float(start_wp["z"]),
            psi=float(start_wp["psi"]),
            theta=0.0,
        )

        self.retreat_path = self._build_segment(
            start_wp=start_wp,
            end_wp=end_wp,
            duration_sec=self.leave_time_sec,
        )
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
            self.waypoint_index = 0
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
                self._publish_waypoint(wp, zero_velocity=True)
                if self.approach_index < len(self.approach_path) - 1:
                    self.approach_index += 1
                return
            self._publish_hover()
            return

        if self.phase == TaskPhase.INITIAL_CONTACT:
            if self.initial_contact_active and self.initial_contact_index < len(self.initial_contact_path):
                wp = self.initial_contact_path[self.initial_contact_index]
                self._publish_waypoint(wp, zero_velocity=True)
                if self.initial_contact_index < len(self.initial_contact_path) - 1:
                    self.initial_contact_index += 1
                return
            self._publish_hover()
            return

        if self.phase == TaskPhase.SLIDING_CONTACT:
            wp = self.waypoints[min(self.waypoint_index, len(self.waypoints) - 1)]
            self._publish_waypoint(wp)
            self._advance_waypoint()
            return

        if self.phase == TaskPhase.RETREAT:
            if self.retreat_active and self.retreat_index < len(self.retreat_path):
                wp = self.retreat_path[self.retreat_index]
                self._publish_waypoint(wp, zero_velocity=True)
                if self.retreat_index < len(self.retreat_path) - 1:
                    self.retreat_index += 1
                return
            self._publish_hover()
            return


def main():
    rospy.init_node("trajectory_server", anonymous=False)

    path_csv = rospy.get_param("/trajectory_server/path_csv", "")
    publish_rate_hz = float(rospy.get_param("/trajectory_server/publish_rate_hz", 20.0))
    leave_time_sec = float(rospy.get_param("/trajectory_server/leave_time_sec", 20.0))
    approach_offset_m = float(rospy.get_param("/trajectory_server/approach_offset_m", 0.3))
    approach_time_sec = float(rospy.get_param("/trajectory_server/approach_time_sec", 30.0))
    initial_contact_time_sec = float(rospy.get_param("/trajectory_server/initial_contact_time_sec", 5.0))

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

    server = TrajectoryServer(
        trajectory_publisher=trajectory_publisher,
        joint_publisher=joint_publisher,
        publish_rate_hz=publish_rate_hz,
        leave_time_sec=leave_time_sec,
        approach_offset_m=approach_offset_m,
        approach_time_sec=approach_time_sec,
        initial_contact_time_sec=initial_contact_time_sec,
    )

    try:
        waypoints = server.load_csv(resolved_csv_path)
        server.waypoints = waypoints
    except (FileNotFoundError, ValueError) as exc:
        rospy.logerr("Failed to load trajectory CSV: {}".format(exc))
        return

    def _on_task_phase(msg):
        server.set_phase(msg.phase)

    rospy.Subscriber("/uav_contact/task/phase", TaskPhase, _on_task_phase, queue_size=10)

    rate = rospy.Rate(publish_rate_hz)
    rospy.loginfo("Trajectory server started at {} Hz".format(publish_rate_hz))

    while not rospy.is_shutdown():
        server.publish()
        rate.sleep()


if __name__ == "__main__":
    main()
