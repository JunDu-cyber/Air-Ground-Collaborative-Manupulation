#!/usr/bin/env python3
"""Debug-only alignment check for LiDAR detection, estimator state, and truth."""

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32


class LidarDetectionAlignmentCheck:
    def __init__(self):
        self.target_observation_topic = rospy.get_param(
            "~target_observation_topic", "/target_observation/pose"
        )
        self.estimator_state_topic = rospy.get_param(
            "~estimator_state_topic", "/target_estimator/state"
        )
        self.uav1_odom_topic = rospy.get_param(
            "~uav1_odom_topic", "/uav1/mavros/local_position/odom"
        )
        self.use_spawn_offsets = rospy.get_param("~use_spawn_offsets", True)
        self.uav1_spawn_offset = (
            rospy.get_param("~uav1_spawn_x", 2.0),
            rospy.get_param("~uav1_spawn_y", 0.0),
            rospy.get_param("~uav1_spawn_z", 0.0),
        )
        self.warn_distance = rospy.get_param("~warn_distance", 1.0)
        self.offset_warn_threshold = rospy.get_param("~offset_warn_threshold", 1.0)

        self.detection = None
        self.estimator = None
        self.truth = None

        self.det_truth_pub = rospy.Publisher(
            "/target_observation/alignment/detection_to_truth_distance",
            Float32,
            queue_size=5,
        )
        self.est_truth_pub = rospy.Publisher(
            "/target_observation/alignment/estimator_to_truth_distance",
            Float32,
            queue_size=5,
        )
        self.det_est_pub = rospy.Publisher(
            "/target_observation/alignment/detection_to_estimator_distance",
            Float32,
            queue_size=5,
        )
        self.offset_warning_pub = rospy.Publisher(
            "/target_observation/alignment/spawn_offset_warning",
            Bool,
            queue_size=5,
        )

        rospy.Subscriber(
            self.target_observation_topic,
            PoseStamped,
            self.detection_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.estimator_state_topic,
            Odometry,
            self.estimator_cb,
            queue_size=20,
        )
        rospy.Subscriber(self.uav1_odom_topic, Odometry, self.truth_cb, queue_size=20)

        rospy.logwarn(
            "[LidarDetectionAlignmentCheck] debug-only node uses /uav1 truth for offline validation; do not feed this output into detector/estimator"
        )

    @staticmethod
    def _dist(a, b):
        return math.sqrt(
            (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
        )

    def _truth_to_common(self, pos):
        if not self.use_spawn_offsets:
            return pos
        return (
            pos[0] + self.uav1_spawn_offset[0],
            pos[1] + self.uav1_spawn_offset[1],
            pos[2] + self.uav1_spawn_offset[2],
        )

    def detection_cb(self, msg):
        p = msg.pose.position
        self.detection = (p.x, p.y, p.z)

    def estimator_cb(self, msg):
        p = msg.pose.pose.position
        self.estimator = (p.x, p.y, p.z)

    def truth_cb(self, msg):
        p = msg.pose.pose.position
        self.truth = self._truth_to_common((p.x, p.y, p.z))

    def run(self):
        rate = rospy.Rate(rospy.get_param("~publish_rate", 5.0))
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        if self.truth is None:
            rospy.loginfo_throttle(
                3.0, "[LidarDetectionAlignmentCheck] waiting for /uav1 truth odom"
            )
            return

        det_truth = float("nan")
        est_truth = float("nan")
        det_est = float("nan")
        if self.detection is not None:
            det_truth = self._dist(self.detection, self.truth)
        if self.estimator is not None:
            est_truth = self._dist(self.estimator, self.truth)
        if self.detection is not None and self.estimator is not None:
            det_est = self._dist(self.detection, self.estimator)

        self.det_truth_pub.publish(Float32(data=det_truth))
        self.est_truth_pub.publish(Float32(data=est_truth))
        self.det_est_pub.publish(Float32(data=det_est))

        offset_warning = (
            math.isfinite(det_truth)
            and math.isfinite(det_est)
            and det_truth > self.offset_warn_threshold
            and det_est < self.offset_warn_threshold
        )
        self.offset_warning_pub.publish(Bool(data=offset_warning))

        rospy.loginfo_throttle(
            1.0,
            "[LidarDetectionAlignmentCheck] det_truth=%.2f est_truth=%.2f det_est=%.2f offset_warning=%s",
            det_truth,
            est_truth,
            det_est,
            offset_warning,
        )
        if math.isfinite(det_truth) and det_truth > self.warn_distance:
            rospy.logwarn_throttle(
                2.0,
                "[LidarDetectionAlignmentCheck] detection is %.2fm from truth; check cluster association, frame transform, or spawn offset",
                det_truth,
            )


def main():
    rospy.init_node("check_lidar_detection_alignment")
    LidarDetectionAlignmentCheck().run()


if __name__ == "__main__":
    main()
