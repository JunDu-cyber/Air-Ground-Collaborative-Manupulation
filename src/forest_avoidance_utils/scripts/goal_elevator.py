#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Force RViz 2D goals onto a fixed low-altitude forest test plane."""

import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('goal_elevator')
    goal_z = rospy.get_param("~goal_z", 1.0)
    input_topic = rospy.get_param("~input_topic", "/move_base_simple/goal")
    output_topic = rospy.get_param("~output_topic", "/goal_elevated")

    pub = rospy.Publisher(output_topic, PoseStamped, queue_size=1)
    receive_count = [0]
    drone_z = [0.0]

    def odom_cb(msg):
        drone_z[0] = msg.pose.position.z

    def goal_cb(msg):
        receive_count[0] += 1
        old_z = msg.pose.position.z
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.z = goal_z
        pub.publish(msg)
        rospy.logwarn(
            "[GoalElevator] #%d: (%.1f,%.1f) old_z=%.1f drone_z=%.1f -> goal_z=%.1f, publish now",
            receive_count[0], msg.pose.position.x, msg.pose.position.y,
            old_z, drone_z[0], goal_z)

    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, odom_cb)
    rospy.Subscriber(input_topic, PoseStamped, goal_cb)

    rospy.logwarn(
        "[GoalElevator] Ready (sub=%s, pub=%s, fixed_goal_z=%.2f, no release gate)",
        input_topic, output_topic, goal_z)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
