#!/usr/bin/env python3
"""Debug-only live comparison of target observations against UAV1 truth."""

import math

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray


class PoseSample:
    def __init__(self):
        self.pos = None
        self.stamp = None
        self.valid = True
        self.confidence = float("nan")
        self.source = ""


class TargetTruthCompare:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.use_spawn_offsets = rospy.get_param("~use_spawn_offsets", True)
        self.uav1_spawn_offset = (
            rospy.get_param("~uav1_spawn_x", 2.0),
            rospy.get_param("~uav1_spawn_y", 0.0),
            rospy.get_param("~uav1_spawn_z", 0.0),
        )
        self.max_age = max(rospy.get_param("~max_age", 0.8), 0.0)
        self.publish_rate = max(rospy.get_param("~publish_rate", 2.0), 0.2)

        self.uav1_truth = PoseSample()
        self.visual = PoseSample()
        self.lidar = PoseSample()
        self.fused = PoseSample()
        self.estimator = PoseSample()
        self.intercept = PoseSample()

        self.marker_pub = rospy.Publisher(
            rospy.get_param("~marker_topic", "/target_truth_compare/marker"),
            MarkerArray,
            queue_size=5,
        )

        rospy.Subscriber(
            rospy.get_param("~uav1_odom_topic", "/uav1/mavros/local_position/odom"),
            Odometry,
            self.uav1_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~visual_pose_topic", "/target_visual/pose"),
            PoseStamped,
            lambda msg: self.pose_cb(msg, self.visual),
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~visual_valid_topic", "/target_visual/valid"),
            Bool,
            lambda msg: self.valid_cb(msg, self.visual),
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~visual_confidence_topic", "/target_visual/confidence"),
            Float32,
            lambda msg: self.confidence_cb(msg, self.visual),
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~lidar_pose_topic", "/target_observation/pose"),
            PoseStamped,
            lambda msg: self.pose_cb(msg, self.lidar),
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~lidar_valid_topic", "/target_observation/valid"),
            Bool,
            lambda msg: self.valid_cb(msg, self.lidar),
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~lidar_confidence_topic", "/target_observation/confidence"),
            Float32,
            lambda msg: self.confidence_cb(msg, self.lidar),
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~fused_pose_topic", "/target_fused/pose"),
            PoseStamped,
            lambda msg: self.pose_cb(msg, self.fused),
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~fused_valid_topic", "/target_fused/valid"),
            Bool,
            lambda msg: self.valid_cb(msg, self.fused),
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~fused_confidence_topic", "/target_fused/confidence"),
            Float32,
            lambda msg: self.confidence_cb(msg, self.fused),
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~fused_source_topic", "/target_fused/source"),
            String,
            self.fused_source_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~estimator_state_topic", "/target_estimator/state"),
            Odometry,
            self.estimator_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~intercept_topic", "/target_estimator/intercept_point"),
            PoseStamped,
            lambda msg: self.pose_cb(msg, self.intercept),
            queue_size=20,
        )

        rospy.logwarn(
            "[TargetTruthCompare] DEBUG ONLY: subscribes to /uav1 odom for live error comparison; not used by detector/fusion/estimator."
        )

    def _to_common(self, pos):
        if not self.use_spawn_offsets:
            return pos
        return (
            pos[0] + self.uav1_spawn_offset[0],
            pos[1] + self.uav1_spawn_offset[1],
            pos[2] + self.uav1_spawn_offset[2],
        )

    @staticmethod
    def _dist(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def _fmt_pos(pos):
        if pos is None:
            return "(nan nan nan)"
        return "({:.2f} {:.2f} {:.2f})".format(pos[0], pos[1], pos[2])

    def uav1_cb(self, msg):
        p = msg.pose.pose.position
        self.uav1_truth.pos = self._to_common((p.x, p.y, p.z))
        self.uav1_truth.stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

    def pose_cb(self, msg, sample):
        p = msg.pose.position
        sample.pos = (p.x, p.y, p.z)
        sample.stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

    def estimator_cb(self, msg):
        p = msg.pose.pose.position
        self.estimator.pos = (p.x, p.y, p.z)
        self.estimator.stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

    @staticmethod
    def valid_cb(msg, sample):
        sample.valid = bool(msg.data)

    @staticmethod
    def confidence_cb(msg, sample):
        sample.confidence = float(msg.data)

    def fused_source_cb(self, msg):
        self.fused.source = msg.data.strip().lower()

    def _fresh(self, sample, now):
        if sample.pos is None or not sample.valid:
            return False
        if sample.stamp is None or self.max_age <= 0.0:
            return True
        return 0.0 <= (now - sample.stamp).to_sec() <= self.max_age

    def _err(self, sample, now):
        if self.uav1_truth.pos is None or not self._fresh(sample, now):
            return float("nan")
        return self._dist(sample.pos, self.uav1_truth.pos)

    def _line_marker(self, marker_id, name, sample, color, now):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = now
        marker.ns = "target_truth_compare_" + name
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.035
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime = rospy.Duration(0.8)
        if self.uav1_truth.pos is not None and sample.pos is not None:
            a = Point()
            a.x, a.y, a.z = self.uav1_truth.pos
            b = Point()
            b.x, b.y, b.z = sample.pos
            marker.points.append(a)
            marker.points.append(b)
        return marker

    def _truth_marker(self, now):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = now
        marker.ns = "target_truth_compare_truth"
        marker.id = 100
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        if self.uav1_truth.pos is not None:
            marker.pose.position.x = self.uav1_truth.pos[0]
            marker.pose.position.y = self.uav1_truth.pos[1]
            marker.pose.position.z = self.uav1_truth.pos[2]
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.22
        marker.color.r = 1.0
        marker.color.g = 0.35
        marker.color.b = 0.1
        marker.color.a = 0.95
        marker.lifetime = rospy.Duration(0.8)
        return marker

    def publish(self):
        now = rospy.Time.now()
        visual_err = self._err(self.visual, now)
        lidar_err = self._err(self.lidar, now)
        fused_err = self._err(self.fused, now)
        estimator_err = self._err(self.estimator, now)
        intercept_err = self._err(self.intercept, now)
        rospy.loginfo(
            "[TargetTruthCompare] truth=%s visual=%s err=%.2f valid=%s conf=%.2f "
            "lidar=%s err=%.2f valid=%s conf=%.2f fused=%s err=%.2f valid=%s source=%s conf=%.2f "
            "estimator=%s err=%.2f intercept=%s now_err=%.2f",
            self._fmt_pos(self.uav1_truth.pos),
            self._fmt_pos(self.visual.pos),
            visual_err,
            self.visual.valid,
            self.visual.confidence,
            self._fmt_pos(self.lidar.pos),
            lidar_err,
            self.lidar.valid,
            self.lidar.confidence,
            self._fmt_pos(self.fused.pos),
            fused_err,
            self.fused.valid,
            self.fused.source or "none",
            self.fused.confidence,
            self._fmt_pos(self.estimator.pos),
            estimator_err,
            self._fmt_pos(self.intercept.pos),
            intercept_err,
        )

        markers = MarkerArray()
        markers.markers.append(self._truth_marker(now))
        markers.markers.append(self._line_marker(0, "visual", self.visual, (0.0, 1.0, 0.2, 0.85), now))
        markers.markers.append(self._line_marker(1, "lidar", self.lidar, (0.2, 0.55, 1.0, 0.85), now))
        markers.markers.append(self._line_marker(2, "fused", self.fused, (1.0, 0.8, 0.0, 0.85), now))
        markers.markers.append(self._line_marker(3, "estimator", self.estimator, (0.0, 1.0, 1.0, 0.85), now))
        self.marker_pub.publish(markers)

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


def main():
    rospy.init_node("target_truth_compare_node")
    TargetTruthCompare().run()


if __name__ == "__main__":
    main()
