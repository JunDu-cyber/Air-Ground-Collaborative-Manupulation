#!/usr/bin/env python3
"""Continuously send overhead scan goals around truth-known mines for data collection."""

import math
import random
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped


class MineWaypoints:
    def __init__(self):
        self.pub = rospy.Publisher(rospy.get_param("~goal_topic", "/move_base_simple/goal"),
                                   PoseStamped, queue_size=1, latch=True)
        self.dwell = float(rospy.get_param("~dwell", 6.0))
        self.jitter = float(rospy.get_param("~jitter", 0.7))
        self.negative_goal_ratio = float(rospy.get_param("~negative_goal_ratio", 0.30))
        self.rng = random.Random(int(rospy.get_param("~seed", 43)))
        self.positions = []
        self.negative_positions = []
        self.index = 0
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.cb, queue_size=1)
        rospy.Timer(rospy.Duration(self.dwell), self.tick)

    def cb(self, msg):
        pts = [(p.position.x, p.position.y) for n, p in zip(msg.name, msg.pose)
               if n == "landmine" or n.startswith("landmine_")]
        negatives = [(p.position.x, p.position.y) for n, p in zip(msg.name, msg.pose)
                     if n.startswith("distractor_") or n.startswith("rubble_")]
        if pts:
            self.positions = sorted(pts)
        if negatives:
            self.negative_positions = sorted(negatives)

    def tick(self, _event):
        if not self.positions:
            rospy.logwarn_throttle(5.0, "[MineWaypoints] waiting for landmines")
            return
        use_negative = self.negative_positions and self.rng.random() < self.negative_goal_ratio
        source = self.negative_positions if use_negative else self.positions
        x, y = source[self.index % len(source)]
        self.index += 1
        x += self.rng.uniform(-self.jitter, self.jitter)
        y += self.rng.uniform(-self.jitter, self.jitter)
        msg = PoseStamped(); msg.header.stamp = rospy.Time.now(); msg.header.frame_id = "map"
        msg.pose.position.x=x; msg.pose.position.y=y
        yaw = self.rng.uniform(-math.pi, math.pi)
        msg.pose.orientation.z=math.sin(yaw/2.0); msg.pose.orientation.w=math.cos(yaw/2.0)
        self.pub.publish(msg)
        rospy.loginfo("[MineWaypoints] goal=(%.2f, %.2f) kind=%s", x, y,
                      "negative" if use_negative else "mine")


if __name__ == "__main__":
    rospy.init_node("mine_collection_waypoints")
    MineWaypoints()
    rospy.spin()
