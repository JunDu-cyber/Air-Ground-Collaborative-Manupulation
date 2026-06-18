#!/usr/bin/env python3
"""Decide whether the pursuing UAV has captured the target UAV."""

import math

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


class CaptureDecision:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.uav0_odom_topic = rospy.get_param(
            "~uav0_odom_topic", "/uav0/mavros/local_position/odom"
        )
        self.uav1_odom_topic = rospy.get_param(
            "~uav1_odom_topic", "/uav1/mavros/local_position/odom"
        )
        self.success_topic = rospy.get_param("~success_topic", "/uav0/capture/success")
        self.marker_topic = rospy.get_param("~marker_topic", "/uav0/capture/marker")
        self.capture_distance = rospy.get_param("~capture_distance", 1.0)
        self.hard_capture_distance = rospy.get_param(
            "~hard_capture_distance", max(0.0, 0.7 * self.capture_distance)
        )
        self.capture_use_3d_distance = rospy.get_param("~capture_use_3d_distance", True)
        self.hold_time = rospy.get_param("~hold_time", 1.0)
        self.latch_success = rospy.get_param("~latch_success", True)
        self.use_spawn_offsets = rospy.get_param("~use_spawn_offsets", True)
        self.uav0_spawn_offset = (
            rospy.get_param("~uav0_spawn_x", 0.0),
            rospy.get_param("~uav0_spawn_y", 0.0),
            rospy.get_param("~uav0_spawn_z", 0.0),
        )
        self.uav1_spawn_offset = (
            rospy.get_param("~uav1_spawn_x", 2.0),
            rospy.get_param("~uav1_spawn_y", 0.0),
            rospy.get_param("~uav1_spawn_z", 0.0),
        )

        self.uav0_pose = None
        self.uav1_pose = None
        self.inside_since = None
        self.success_latched = False

        self.uav0_sub = rospy.Subscriber(
            self.uav0_odom_topic, Odometry, self.uav0_cb, queue_size=10
        )
        self.uav1_sub = rospy.Subscriber(
            self.uav1_odom_topic, Odometry, self.uav1_cb, queue_size=10
        )
        self.success_pub = rospy.Publisher(self.success_topic, Bool, queue_size=10)
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=10)

        rospy.loginfo(
            "[CaptureDecision] uav0=%s uav1=%s success=%s marker=%s capture_distance=%.2f hard_capture_distance=%.2f capture_use_3d=%s hold_time=%.2f latch_success=%s offsets=%s uav0_offset=(%.2f %.2f %.2f) uav1_offset=(%.2f %.2f %.2f)",
            self.uav0_odom_topic,
            self.uav1_odom_topic,
            self.success_topic,
            self.marker_topic,
            self.capture_distance,
            self.hard_capture_distance,
            self.capture_use_3d_distance,
            self.hold_time,
            self.latch_success,
            self.use_spawn_offsets,
            self.uav0_spawn_offset[0],
            self.uav0_spawn_offset[1],
            self.uav0_spawn_offset[2],
            self.uav1_spawn_offset[0],
            self.uav1_spawn_offset[1],
            self.uav1_spawn_offset[2],
        )

    def uav0_cb(self, msg):
        self.uav0_pose = msg.pose.pose

    def uav1_cb(self, msg):
        self.uav1_pose = msg.pose.pose

    @staticmethod
    def distance_metrics(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        distance_xy = math.sqrt(dx * dx + dy * dy)
        distance_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
        return distance_xy, distance_3d

    def common_position(self, pose, offset):
        p = pose.position
        if not self.use_spawn_offsets:
            return p.x, p.y, p.z
        return p.x + offset[0], p.y + offset[1], p.z + offset[2]

    def update_success(self, distance):
        now = rospy.Time.now()
        reason = "outside"
        if self.success_latched:
            held = self.hold_time
            success = True
            reason = "latched"
        elif distance <= self.hard_capture_distance:
            self.inside_since = now
            held = self.hold_time
            success = True
            reason = "hard"
            if self.latch_success:
                self.success_latched = True
        elif distance <= self.capture_distance:
            if self.inside_since is None:
                self.inside_since = now
            held = (now - self.inside_since).to_sec()
            success = held >= self.hold_time
            reason = "hold" if success else "inside"
            if success and self.latch_success:
                self.success_latched = True
        else:
            self.inside_since = None
            held = 0.0
            success = False

        self.success_pub.publish(Bool(data=success))
        rospy.loginfo_throttle(
            1.0,
            "[CaptureDecision] distance=%.2f threshold=%.2f hard_threshold=%.2f use_3d=%s held=%.2f success=%s latched=%s reason=%s",
            distance,
            self.capture_distance,
            self.hard_capture_distance,
            self.capture_use_3d_distance,
            held,
            success,
            self.success_latched,
            reason,
        )
        return success

    def publish_marker(self, uav0, uav1, distance_xy, distance_3d, success):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = "capture_decision"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.06
        marker.color.r = 0.1 if success else 1.0
        active_distance = distance_3d if self.capture_use_3d_distance else distance_xy
        marker.color.g = 1.0 if active_distance <= self.capture_distance else 0.25
        marker.color.b = 0.1
        marker.color.a = 0.9
        marker.lifetime = rospy.Duration(0.25)

        marker.points.append(Point(x=uav0[0], y=uav0[1], z=uav0[2]))
        marker.points.append(Point(x=uav1[0], y=uav1[1], z=uav1[2]))
        self.marker_pub.publish(marker)

        text = Marker()
        text.header = marker.header
        text.ns = "capture_decision"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.orientation.w = 1.0
        text.pose.position.x = 0.5 * (uav0[0] + uav1[0])
        text.pose.position.y = 0.5 * (uav0[1] + uav1[1])
        text.pose.position.z = 0.5 * (uav0[2] + uav1[2]) + 0.45
        text.scale.z = 0.28
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 0.95
        text.text = "distance_xy={:.2f}\ndistance_3d={:.2f}".format(
            distance_xy, distance_3d
        )
        text.lifetime = rospy.Duration(0.25)
        self.marker_pub.publish(text)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.uav0_pose is None or self.uav1_pose is None:
                rospy.loginfo_throttle(3.0, "[CaptureDecision] waiting for both odom topics")
                self.success_pub.publish(Bool(data=False))
                rate.sleep()
                continue

            uav0 = self.common_position(self.uav0_pose, self.uav0_spawn_offset)
            uav1 = self.common_position(self.uav1_pose, self.uav1_spawn_offset)
            distance_xy, distance_3d = self.distance_metrics(uav0, uav1)
            active_distance = distance_3d if self.capture_use_3d_distance else distance_xy
            success = self.update_success(active_distance)
            self.publish_marker(uav0, uav1, distance_xy, distance_3d, success)
            rate.sleep()


def main():
    rospy.init_node("capture_decision_node")
    CaptureDecision().run()


if __name__ == "__main__":
    main()
