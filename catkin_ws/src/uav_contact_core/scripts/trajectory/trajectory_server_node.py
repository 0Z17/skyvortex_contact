#!/usr/bin/env python3

import csv
from pathlib import Path


class TrajectoryServer:
    def __init__(self, publisher=None):
        self.publisher = publisher

    def load_csv(self, csv_path):
        path = Path(csv_path)
        waypoints = []
        with path.open("r", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                waypoints.append(
                    (
                        float(row["x"]),
                        float(row["y"]),
                        float(row["z"]),
                        float(row["yaw"]),
                    )
                )
        return waypoints

    def publish_waypoints(self, waypoints):
        if self.publisher is None:
            return
        self.publisher.publish(waypoints)
