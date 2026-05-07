#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import Range


class LaserDistanceRelayNode:
    def __init__(self):
        rospy.init_node("laser_distance_relay")

        self.init_duration = float(rospy.get_param("/laser_distance_relay/init_duration", 5.0))
        self.laser_topic = rospy.get_param("/laser_distance_relay/laser_topic", "/laser")
        self.distance_topic = rospy.get_param("/laser_distance_relay/distance_topic", "/contact/distance")

        self.range_init = None
        self.init_samples = []
        self.init_complete = False

        self.distance_pub = rospy.Publisher(self.distance_topic, Float64, queue_size=10)
        self.laser_sub = rospy.Subscriber(self.laser_topic, Range, self._on_range)

        self.start_time = rospy.Time.now()

        rospy.loginfo("Laser distance relay started, initializing for %.1f seconds...", self.init_duration)

    def _on_range(self, msg):
        now = rospy.Time.now()
        elapsed = (now - self.start_time).to_sec()

        if not self.init_complete:
            self.init_samples.append(msg.range)
            if elapsed >= self.init_duration:
                if len(self.init_samples) > 0:
                    self.range_init = float(np.mean(self.init_samples))
                    self.init_complete = True
                    rospy.loginfo("Initialization complete, range_init = %.4f (from %d samples)",
                                  self.range_init, len(self.init_samples))
                else:
                    rospy.logerr("No laser samples received during %.1fs init period, shutting down", self.init_duration)
                    rospy.signal_shutdown("No laser data available for initialization")
        else:
            distance = self.range_init - msg.range
            self.distance_pub.publish(Float64(data=distance))

    def spin(self):
        rospy.spin()


def main():
    node = LaserDistanceRelayNode()
    node.spin()


if __name__ == "__main__":
    main()
