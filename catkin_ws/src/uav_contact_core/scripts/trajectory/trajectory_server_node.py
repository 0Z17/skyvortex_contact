#!/usr/bin/env python3

import csv
from pathlib import Path

import rospy


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
        if self.publisher is None:
            return
        self.publisher.publish(waypoints)


def main():
    rospy.init_node("trajectory_server", anonymous=False)
    rospy.loginfo("Trajectory server baseline node started")
    rospy.spin()


if __name__ == "__main__":
    main()
