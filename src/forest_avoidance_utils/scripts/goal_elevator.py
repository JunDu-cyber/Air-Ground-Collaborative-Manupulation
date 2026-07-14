#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Preserve explicit goal altitude; keep current altitude for RViz 2D goals."""

import copy
import math
import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('goal_elevator')
    goal_z = rospy.get_param("~goal_z", 1.0)
    zero_z_epsilon = max(
        0.0, rospy.get_param("~zero_goal_z_epsilon", 0.05))
    input_topic = rospy.get_param("~input_topic", "/move_base_simple/goal")
    output_topic = rospy.get_param("~output_topic", "/goal_elevated")

    pub = rospy.Publisher(output_topic, PoseStamped, queue_size=1)
    receive_count = [0]
    drone_z = [None]

    def odom_cb(msg):
        drone_z[0] = msg.pose.position.z

    def goal_cb(msg):
        receive_count[0] += 1
        old_z = msg.pose.position.z
        if abs(old_z) > zero_z_epsilon:
            resolved_z = old_z
            source = "MESSAGE_Z"
        elif drone_z[0] is not None and math.isfinite(drone_z[0]):
            resolved_z = drone_z[0]
            source = "CURRENT_Z"
        else:
            resolved_z = goal_z
            source = "FALLBACK_Z"
        output = copy.deepcopy(msg)
        output.header.stamp = rospy.Time.now()
        output.pose.position.z = resolved_z
        pub.publish(output)
        rospy.logwarn(
            "[GoalElevator] #%d: (%.1f,%.1f) input_z=%.2f current_z=%s -> "
            "goal_z=%.2f source=%s",
            receive_count[0], msg.pose.position.x, msg.pose.position.y,
            old_z, "unknown" if drone_z[0] is None else "%.2f" % drone_z[0],
            resolved_z, source)

    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, odom_cb)
    rospy.Subscriber(input_topic, PoseStamped, goal_cb)

    rospy.logwarn(
        "[GoalElevator] Ready (sub=%s, pub=%s, fallback_z=%.2f; "
        "explicit z preserved, 2D goals hold current z)",
        input_topic, output_topic, goal_z)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
