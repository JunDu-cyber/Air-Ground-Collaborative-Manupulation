#!/usr/bin/env python3
"""Classify visual-loss states and publish a yaw target for reacquisition."""

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String


class TargetReacquireManager:
    def __init__(self):
        self.visual_valid_topic = rospy.get_param(
            "~visual_valid_topic", "/target_visual/valid"
        )
        self.fused_valid_topic = rospy.get_param(
            "~fused_valid_topic", "/target_fused/valid"
        )
        self.fused_source_topic = rospy.get_param(
            "~fused_source_topic", "/target_fused/source"
        )
        self.fused_pose_topic = rospy.get_param(
            "~fused_pose_topic", "/target_fused/pose"
        )
        self.estimator_state_topic = rospy.get_param(
            "~estimator_state_topic", "/target_estimator/state"
        )
        self.tracking_state_topic = rospy.get_param(
            "~tracking_state_topic", "/target_estimator/tracking_state"
        )
        self.uav0_odom_topic = rospy.get_param(
            "~uav0_odom_topic", "/uav0/mavros/local_position/odom"
        )
        self.state_topic = rospy.get_param(
            "~state_topic", "/target_reacquire/state"
        )
        self.yaw_target_topic = rospy.get_param(
            "~yaw_target_topic", "/target_reacquire/yaw_target"
        )

        self.visual_lost_timeout = max(
            rospy.get_param("~visual_lost_timeout", 0.3), 0.0
        )
        self.full_lost_timeout = max(
            rospy.get_param("~full_lost_timeout", 1.5), self.visual_lost_timeout
        )
        self.predict_only_speed_scale = rospy.get_param(
            "~predict_only_speed_scale", 0.6
        )
        self.reacquire_yaw_scan_rate = rospy.get_param(
            "~reacquire_yaw_scan_rate", 0.3
        )
        self.use_yaw_reacquire = rospy.get_param("~use_yaw_reacquire", True)
        self.publish_rate = max(rospy.get_param("~publish_rate", 20.0), 1.0)

        self.visual_valid = False
        self.visual_stamp = None
        self.fused_valid = False
        self.fused_valid_stamp = None
        self.fused_source = "none"
        self.fused_source_stamp = None
        self.fused_pos = None
        self.fused_stamp = None
        self.estimator_pos = None
        self.estimator_stamp = None
        self.tracking_state = "LOST"
        self.tracking_state_stamp = None
        self.uav0_pos = None
        self.uav0_yaw = 0.0
        self.scan_yaw = None
        self.last_publish = rospy.Time.now()

        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=5)
        self.yaw_target_pub = rospy.Publisher(
            self.yaw_target_topic, Float32, queue_size=5
        )

        rospy.Subscriber(self.visual_valid_topic, Bool, self.visual_valid_cb, queue_size=10)
        rospy.Subscriber(self.fused_valid_topic, Bool, self.fused_valid_cb, queue_size=10)
        rospy.Subscriber(self.fused_source_topic, String, self.fused_source_cb, queue_size=10)
        rospy.Subscriber(self.fused_pose_topic, PoseStamped, self.fused_pose_cb, queue_size=10)
        rospy.Subscriber(
            self.estimator_state_topic, Odometry, self.estimator_state_cb, queue_size=10
        )
        rospy.Subscriber(
            self.tracking_state_topic, String, self.tracking_state_cb, queue_size=10
        )
        rospy.Subscriber(self.uav0_odom_topic, Odometry, self.uav0_odom_cb, queue_size=20)

        rospy.loginfo(
            "[TargetReacquireManager] visual_valid=%s fused=(%s %s %s) estimator=%s "
            "state_out=%s yaw_out=%s timeouts=(visual %.2f full %.2f) yaw_scan=%.2f use_yaw=%s",
            self.visual_valid_topic,
            self.fused_pose_topic,
            self.fused_valid_topic,
            self.fused_source_topic,
            self.estimator_state_topic,
            self.state_topic,
            self.yaw_target_topic,
            self.visual_lost_timeout,
            self.full_lost_timeout,
            self.reacquire_yaw_scan_rate,
            self.use_yaw_reacquire,
        )

    @staticmethod
    def _quat_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def visual_valid_cb(self, msg):
        self.visual_valid = bool(msg.data)
        self.visual_stamp = rospy.Time.now()

    def fused_valid_cb(self, msg):
        self.fused_valid = bool(msg.data)
        self.fused_valid_stamp = rospy.Time.now()

    def fused_source_cb(self, msg):
        self.fused_source = msg.data.strip().lower() or "none"
        self.fused_source_stamp = rospy.Time.now()

    def fused_pose_cb(self, msg):
        p = msg.pose.position
        self.fused_pos = (p.x, p.y, p.z)
        self.fused_stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

    def estimator_state_cb(self, msg):
        p = msg.pose.pose.position
        self.estimator_pos = (p.x, p.y, p.z)
        self.estimator_stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

    def tracking_state_cb(self, msg):
        self.tracking_state = msg.data.strip().upper() or "LOST"
        self.tracking_state_stamp = rospy.Time.now()

    def uav0_odom_cb(self, msg):
        p = msg.pose.pose.position
        self.uav0_pos = (p.x, p.y, p.z)
        self.uav0_yaw = self._quat_to_yaw(msg.pose.pose.orientation)

    def _fresh(self, stamp, now, timeout):
        return stamp is not None and (now - stamp).to_sec() <= timeout

    def _yaw_to_position(self, target_pos):
        if self.uav0_pos is None or target_pos is None:
            return None
        dx = target_pos[0] - self.uav0_pos[0]
        dy = target_pos[1] - self.uav0_pos[1]
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return self.uav0_yaw
        return math.atan2(dy, dx)

    def _scan_yaw_target(self, now):
        if self.scan_yaw is None:
            self.scan_yaw = self.uav0_yaw
        dt = max((now - self.last_publish).to_sec(), 0.0)
        self.scan_yaw = self._wrap_angle(
            self.scan_yaw + self.reacquire_yaw_scan_rate * dt
        )
        return self.scan_yaw

    def _classify(self, now):
        visual_fresh = self.visual_valid and self._fresh(
            self.visual_stamp, now, self.visual_lost_timeout
        )
        fused_fresh = self.fused_valid and self._fresh(
            self.fused_valid_stamp, now, self.visual_lost_timeout
        )
        fused_is_lidar = fused_fresh and self.fused_source == "lidar"
        estimator_fresh = self.estimator_pos is not None and self._fresh(
            self.estimator_stamp, now, self.full_lost_timeout
        )

        if visual_fresh or (fused_fresh and self.fused_source == "visual"):
            self.scan_yaw = None
            return "VISUAL_TRACK", self._yaw_to_position(self.fused_pos or self.estimator_pos)
        if fused_is_lidar and self.fused_pos is not None:
            self.scan_yaw = None
            return "LIDAR_REACQUIRE", self._yaw_to_position(self.fused_pos)
        if estimator_fresh and self.tracking_state in ("TRACKING", "PREDICT_ONLY"):
            self.scan_yaw = None
            return "VISUAL_LOST_PREDICT", self._yaw_to_position(self.estimator_pos)
        return "FULL_LOST", self._scan_yaw_target(now)

    def publish(self):
        now = rospy.Time.now()
        state, yaw_target = self._classify(now)
        self.state_pub.publish(String(data=state))
        if self.use_yaw_reacquire and yaw_target is not None:
            self.yaw_target_pub.publish(Float32(data=yaw_target))
        self.last_publish = now
        rospy.loginfo_throttle(
            1.0,
            "[TargetReacquireManager] state=%s visual_valid=%s fused_valid=%s source=%s tracking=%s yaw_target=%s predict_speed_scale=%.2f",
            state,
            self.visual_valid,
            self.fused_valid,
            self.fused_source,
            self.tracking_state,
            "{:.2f}".format(yaw_target) if yaw_target is not None else "none",
            self.predict_only_speed_scale,
        )

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


def main():
    rospy.init_node("target_reacquire_manager_node")
    TargetReacquireManager().run()


if __name__ == "__main__":
    main()
