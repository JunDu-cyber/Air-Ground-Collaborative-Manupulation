#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Intercept /move_base_simple/goal and force a fixed low-altitude goal plane.

Pure sensor mode: no terrain prior, drone flies at current height.
EGO-Planner handles obstacle avoidance from depth camera entirely.
"""
import rospy
from geometry_msgs.msg import PoseStamped

GOAL_Z = 1.0  # RViz 2D click gives Z≈0; force the forest test plane to 1m.


def main():
    rospy.init_node('goal_elevator')
    goal_z = rospy.get_param("~goal_z", GOAL_Z)
    pub = rospy.Publisher('/goal_elevated', PoseStamped, queue_size=1)
    receive_count = [0]
    drone_z = [0.0]

    def publish_goal(msg):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)

    def odom_cb(msg):
        drone_z[0] = msg.pose.position.z

    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, odom_cb)

    def goal_cb(msg):
        receive_count[0] += 1
        old_z = msg.pose.position.z

        rospy.logwarn("[GoalElevator] #%d: (%.1f,%.1f) old_z=%.1f drone_z=%.1f → goal_z=%.1f，立即发布",
                      receive_count[0], msg.pose.position.x, msg.pose.position.y,
                      old_z, drone_z[0], goal_z)

        msg.pose.position.z = goal_z
        publish_goal(msg)

    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_cb)

    rospy.logwarn("[GoalElevator] Ready (sub=/move_base_simple/goal, pub=/goal_elevated, fixed_goal_z=%.2f, no release gate)",
                  goal_z)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
