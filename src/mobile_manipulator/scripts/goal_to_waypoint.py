#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bridge a UGV goal (PoseStamped) to the CMU local_planner waypoint.

The CMU localPlanner consumes geometry_msgs/PointStamped on /way_point and reads
its x,y as the goal in the world (odom) frame. This node converts Pose->Point and,
if the goal arrives in another frame, TF-transforms it into the target (odom) frame
first.

The input is `/ugv/goal` (NOT `/move_base_simple/goal`): in the air-ground system
`/move_base_simple/goal` is the UAV EGO-planner's goal (goal_elevator.py flies the
drone there), so the UGV must not share it. `/ugv/goal` is the single UGV goal seam,
fed either by ugv_target_tour (the autonomous target sequencer) or by hand for
testing. For solo UGV runs with no UAV, set ~input:=/move_base_simple/goal to reuse
the RViz "2D Nav Goal" tool.

  ~input        (PoseStamped)  default /ugv/goal
  ~output       (PointStamped) default /way_point
  ~target_frame                default odom
"""

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped


def main():
    rospy.init_node('goal_to_waypoint')
    input_topic = rospy.get_param('~input', '/ugv/goal')
    output_topic = rospy.get_param('~output', '/way_point')
    target_frame = rospy.get_param('~target_frame', 'odom')

    pub = rospy.Publisher(output_topic, PointStamped, queue_size=1)

    import tf2_ros
    import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transform)
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    def cb(msg):
        src = msg.header.frame_id or '(none)'
        if target_frame and msg.header.frame_id and msg.header.frame_id != target_frame:
            try:
                msg.header.stamp = rospy.Time(0)  # latest available
                msg = tf_buffer.transform(msg, target_frame, timeout=rospy.Duration(0.5))
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn('[goal_to_waypoint] TF %s->%s failed (%s); using raw',
                              src, target_frame, exc)

        pt = PointStamped()
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = target_frame
        pt.point.x = msg.pose.position.x
        pt.point.y = msg.pose.position.y
        pt.point.z = msg.pose.position.z
        pub.publish(pt)
        rospy.loginfo('[goal_to_waypoint] goal (%.2f, %.2f) frame=%s -> /way_point',
                      pt.point.x, pt.point.y, target_frame)

    rospy.Subscriber(input_topic, PoseStamped, cb)
    rospy.loginfo('[goal_to_waypoint] %s -> %s (frame %s)',
                  input_topic, output_topic, target_frame)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
