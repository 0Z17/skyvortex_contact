#!/usr/bin/env python3

import csv
from pathlib import Path

try:
    import rospy
    from geometry_msgs.msg import Pose, Twist, Vector3, Wrench
    from uav_contact_msgs.msg import TrajectoryPoint
except ImportError:  # pragma: no cover - allows tests without ROS runtime
    rospy = None

    class _Obj:
        pass

    class Pose:
        def __init__(self):
            self.position = _Obj()
            self.orientation = _Obj()
            self.position.x = 0.0
            self.position.y = 0.0
            self.position.z = 0.0
            self.orientation.w = 1.0

    class Twist:
        pass

    class Vector3:
        pass

    class Wrench:
        pass

    class TrajectoryPoint:
        def __init__(self):
            self.header = _Obj()
            self.header.stamp = None
            self.pose = Pose()
            self.twist = Twist()
            self.acceleration = Vector3()
            self.feedforward_wrench = Wrench()
            self.time_from_start = 0.0


class TrajectoryServer:
    def __init__(self, publisher=None):
        self.publisher = publisher

    def load_csv(self, csv_path):
        path = Path(csv_path)
        if not path.is_file():
            raise FileNotFoundError(f"CSV file not found: {path}")

        waypoints = []
        with path.open("r", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            required_columns = {"x", "y", "z", "yaw"}
            fieldnames = set(reader.fieldnames or [])
            missing_columns = sorted(required_columns - fieldnames)
            if missing_columns:
                raise ValueError(
                    f"Missing required CSV columns: {', '.join(missing_columns)}"
                )

            for row_index, row in enumerate(reader, start=2):
                try:
                    waypoints.append(
                        (
                            float(row["x"]),
                            float(row["y"]),
                            float(row["z"]),
                            float(row["yaw"]),
                        )
                    )
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        f"Non-numeric value in CSV at row {row_index}: {row}"
                    ) from exc

        return waypoints

    def publish_waypoints(self, waypoints):
        if self.publisher is None or not waypoints:
            return

        x = waypoints[0][0]
        y = waypoints[0][1]
        z = waypoints[0][2]
        msg = TrajectoryPoint()
        if rospy is not None:
            msg.header.stamp = rospy.Time.now()
        msg.pose = Pose()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        msg.twist = Twist()
        msg.acceleration = Vector3()
        msg.feedforward_wrench = Wrench()
        msg.time_from_start = 0.0
        self.publisher.publish(msg)


def _resolve_csv_path(path_csv):
    prefix = "$(find uav_contact_core)"
    if path_csv.startswith(prefix):
        package_path = Path(__file__).resolve().parents[2]
        return str(package_path / path_csv[len(prefix):].lstrip("/"))
    return path_csv


def main():
    if rospy is None:
        raise RuntimeError("rospy is required to run trajectory_server_node.py")

    rospy.init_node("trajectory_server", anonymous=False)

    path_csv = rospy.get_param("/trajectory_server/path_csv", "")
    publish_rate_hz = float(rospy.get_param("/trajectory_server/publish_rate_hz", 10.0))

    resolved_csv_path = _resolve_csv_path(path_csv)
    publisher = rospy.Publisher("/uav_contact/trajectory/reference", TrajectoryPoint, queue_size=10)
    server = TrajectoryServer(publisher=publisher)

    try:
        waypoints = server.load_csv(resolved_csv_path)
    except (FileNotFoundError, ValueError) as exc:
        rospy.logerr(f"Failed to load trajectory CSV: {exc}")
        return

    rate = rospy.Rate(publish_rate_hz)
    rospy.loginfo("Trajectory server baseline node started")
    while not rospy.is_shutdown():
        server.publish_waypoints(waypoints)
        rate.sleep()


if __name__ == "__main__":
    main()
