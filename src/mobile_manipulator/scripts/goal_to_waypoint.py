#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bridge an RViz "2D Nav Goal" to the CMU local_planner waypoint.

The CMU localPlanner consumes geometry_msgs/PointStamped on /way_point and reads
its x,y as the goal in the world (odom) frame. RViz's "2D Nav Goal" tool emits a
geometry_msgs/PoseStamped on /move_base_simple/goal in the RViz fixed frame. This
node converts Pose->Point and, if the goal arrives in another frame, TF-transforms
it into the target (odom) frame first so clicking "just works".

  ~input        (PoseStamped)  default /move_base_simple/goal
  ~output       (PointStamped) default /way_point
  ~target_frame                default odom
"""

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped


def main():
    rospy.init_node('goal_to_waypoint')
    input_topic = rospy.get_param('~input', '/move_base_simple/goal')
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
