#!/usr/bin/env python3
"""Standalone ROS integration smoke test for the UAV global planner."""

import math
import sys
import threading
import time

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class CorridorSmokeTest(object):
    def __init__(self):
        self.event = threading.Event()
        self.subgoal_event = threading.Event()
        self.path = None
        self.subgoal = None
        self.require_straight_subgoal = False
        self.cloud_pub = rospy.Publisher("/test/points", PointCloud2, queue_size=1, latch=True)
        self.odom_pub = rospy.Publisher("/test/odom", Odometry, queue_size=5)
        self.goal_pub = rospy.Publisher("/test/goal", PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber("/global_path", Path, self.path_cb, queue_size=1)
        rospy.Subscriber("/test/subgoal", PoseStamped, self.subgoal_cb, queue_size=1)

    def path_cb(self, msg):
        if msg.poses:
            self.path = msg
            self.event.set()

    def subgoal_cb(self, msg):
        self.subgoal = msg
        if (not self.require_straight_subgoal
                or abs(msg.pose.position.x) <= 0.75):
            self.subgoal_event.set()

    @staticmethod
    def make_cloud(block_corridor=False):
        points = []
        # Dense ground support is required by the planner's relative-height test.
        for yi in range(-70, 31):
            y = yi * 0.5
            for xi in range(-20, 21):
                points.append((xi * 0.5, y, 0.0))
        # Two buildings leave a 3 m-wide vertical passage around x=0.
        for yi in range(-40, 11):
            y = yi * 0.5
            for xi in list(range(-16, -2)) + list(range(3, 17)):
                x = xi * 0.5
                points.append((x, y, 4.0))
        if block_corridor:
            # Simulate a short-lived smeared return joining both buildings.
            for yi in range(-15, -12):
                for xi in range(-2, 3):
                    points.append((xi * 0.5, yi * 0.5, 4.0))
        return pc2.create_cloud_xyz32(Header(frame_id="map", stamp=rospy.Time.now()), points)

    @staticmethod
    def make_odom():
        msg = Odometry()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = -30.0
        msg.pose.pose.position.z = 4.0
        msg.pose.pose.orientation.w = 1.0
        return msg

    @staticmethod
    def make_goal():
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = 0.0
        msg.pose.position.y = 10.0
        msg.pose.position.z = 4.0
        msg.pose.orientation.w = 1.0
        return msg

    def run(self):
        deadline = time.time() + 3.0
        while (self.cloud_pub.get_num_connections() == 0
               or self.odom_pub.get_num_connections() == 0
               or self.goal_pub.get_num_connections() == 0):
            if time.time() > deadline:
                raise RuntimeError("planner subscribers did not connect")
            time.sleep(0.05)

        blocked_cloud = self.make_cloud(block_corridor=True)
        odom = self.make_odom()
        self.cloud_pub.publish(blocked_cloud)
        for _index in range(10):
            odom.header.stamp = rospy.Time.now()
            self.odom_pub.publish(odom)
            time.sleep(0.05)
        goal = self.make_goal()
        goal.header.stamp = rospy.Time.now()
        self.goal_pub.publish(goal)

        if not self.event.wait(5.0):
            raise RuntimeError("planner did not publish a non-empty path")
        blocked_xy = [(0.0, -30.0)] + [
            (pose.pose.position.x, pose.pose.position.y) for pose in self.path.poses]
        blocked_length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                             for a, b in zip(blocked_xy[:-1], blocked_xy[1:]))
        if blocked_length < 43.0:
            raise AssertionError("temporary bridge should force a detour: %s" % blocked_xy)

        # A clean full snapshot represents the recorder's ray-cleared map.  The
        # planner must forget the temporary bridge and replace the detour with
        # the newly available straight path.
        self.event.clear()
        self.subgoal_event.clear()
        self.path = None
        self.subgoal = None
        self.require_straight_subgoal = True
        time.sleep(0.2)
        self.cloud_pub.publish(self.make_cloud(block_corridor=False))
        if not self.event.wait(5.0):
            raise RuntimeError("planner did not improve the route after map clearing")
        if not self.subgoal_event.wait(2.0):
            raise RuntimeError("planner did not publish a lookahead subgoal")

        xy = [(0.0, -30.0)] + [
            (pose.pose.position.x, pose.pose.position.y) for pose in self.path.poses]
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for a, b in zip(xy[:-1], xy[1:]))
        max_lateral = max(abs(point[0]) for point in xy)
        if length > 41.0 or max_lateral > 0.75:
            raise AssertionError(
                "expected straight corridor path, got length=%.2f max|x|=%.2f poses=%s"
                % (length, max_lateral, xy))
        subgoal_xy = (self.subgoal.pose.position.x, self.subgoal.pose.position.y)
        subgoal_distance = math.hypot(subgoal_xy[0], subgoal_xy[1] + 30.0)
        if not (6.0 <= subgoal_distance <= 7.5) or abs(subgoal_xy[0]) > 0.75:
            raise AssertionError("invalid safe lookahead subgoal: %s distance=%.2f"
                                 % (subgoal_xy, subgoal_distance))
        print("PASS detour %.2fm -> cleared corridor %.2fm, max|x|=%.2f poses=%d subgoal=%s"
              % (blocked_length, length, max_lateral, len(self.path.poses), subgoal_xy))


def main():
    rospy.init_node("global_path_planner_corridor_test", anonymous=True)
    test = CorridorSmokeTest()
    try:
        test.run()
    except Exception as exc:  # noqa: BLE001 - standalone test must report all failures
        print("FAIL: %s" % exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
