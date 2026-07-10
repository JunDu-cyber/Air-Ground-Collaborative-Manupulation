#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bridge /ugv/goal (PoseStamped) -> /goal_point (PointStamped) for FAR planner.

In global_planner:=far mode the FAR planner owns /way_point (the localPlanner
goal), and takes its own goal as a PointStamped on /goal_point. This node keeps
/ugv/goal as the single UGV goal seam (fed by ugv_target_tour or by hand): it
strips the orientation and forwards the position. No TF: FAR consumes the goal
in the frame it runs in (odom), and ugv_target_tour already publishes /ugv/goal
in odom.
"""

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped


def main():
    rospy.init_node('ugv_goal_to_far_goal')
    input_topic = rospy.get_param('~input', '/ugv/goal')
    output_topic = rospy.get_param('~output', '/goal_point')

    pub = rospy.Publisher(output_topic, PointStamped, queue_size=1)

    def cb(msg):
        pt = PointStamped(header=msg.header, point=msg.pose.position)
        pub.publish(pt)
        rospy.loginfo('[ugv_goal_to_far_goal] goal (%.2f, %.2f) frame=%s -> %s',
                      pt.point.x, pt.point.y, msg.header.frame_id or '(none)',
                      output_topic)

    rospy.Subscriber(input_topic, PoseStamped, cb)
    rospy.loginfo('[ugv_goal_to_far_goal] %s -> %s', input_topic, output_topic)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
