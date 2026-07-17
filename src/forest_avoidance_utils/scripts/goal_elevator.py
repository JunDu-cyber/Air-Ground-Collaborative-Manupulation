#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Force RViz 2D goals onto a fixed flight altitude, optionally re-framing them.

The RViz "2D Nav Goal" tool publishes the goal in the RViz Fixed Frame (e.g. the
shared world `map`/`odom`). EGO-Planner, however, plans in the UAV's MAVROS-local
frame and uses the goal's raw x,y numbers WITHOUT transforming them. In the
air-ground co-sim those frames differ by the UAV spawn offset (map -> uav0/map_local
= 0,-18,0), so a goal clicked near the UGV would send the UAV ~18 m off. Set
~target_frame (e.g. uav0/map_local) to TF-transform the clicked goal into the UAV's
local frame first, so clicking in the world map "just works". Leave it empty for the
standalone stack (UAV map == world map), where no transform is needed.
"""

import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('goal_elevator')
    goal_z = rospy.get_param("~goal_z", 1.0)
    input_topic = rospy.get_param("~input_topic", "/move_base_simple/goal")
    legacy_input_topic = rospy.get_param("~legacy_input_topic", "/goal")
    output_topic = rospy.get_param("~output_topic", "/goal_elevated")
    # When set, clicked goals are TF-transformed into this frame before being sent
    # to EGO (which reads the x,y numerically in its MAVROS-local frame).
    target_frame = rospy.get_param("~target_frame", "")

    pub = rospy.Publisher(output_topic, PoseStamped, queue_size=1)
    receive_count = [0]
    drone_z = [0.0]

    tf_buffer = None
    if target_frame:
        import tf2_ros
        import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transform)
        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer)

    def odom_cb(msg):
        drone_z[0] = msg.pose.position.z

    def goal_cb(msg):
        receive_count[0] += 1
        old_z = msg.pose.position.z
        src_frame = msg.header.frame_id or "(none)"

        # Re-frame the clicked goal into the UAV's local frame if requested.
        if tf_buffer is not None and msg.header.frame_id and msg.header.frame_id != target_frame:
            try:
                msg.header.stamp = rospy.Time(0)  # latest available transform
                msg = tf_buffer.transform(msg, target_frame, timeout=rospy.Duration(0.5))
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn(
                    "[GoalElevator] TF %s->%s failed (%s); publishing goal untransformed",
                    src_frame, target_frame, exc)

        msg.header.stamp = rospy.Time.now()
        msg.pose.position.z = goal_z
        pub.publish(msg)
        rospy.logwarn(
            "[GoalElevator] #%d: in=%s (%.1f,%.1f) old_z=%.1f drone_z=%.1f -> out_frame=%s goal_z=%.1f",
            receive_count[0], src_frame, msg.pose.position.x, msg.pose.position.y,
            old_z, drone_z[0], msg.header.frame_id or "(none)", goal_z)

    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, odom_cb)
    rospy.Subscriber(input_topic, PoseStamped, goal_cb)
    # Some saved RViz sessions use rviz/SetGoal's legacy /goal topic. Accept it
    # too so removing the survey arbiter cannot silently break manual flight.
    if legacy_input_topic and legacy_input_topic != input_topic:
        rospy.Subscriber(legacy_input_topic, PoseStamped, goal_cb)

    rospy.logwarn(
        "[GoalElevator] Ready (sub=%s + %s, pub=%s, fixed_goal_z=%.2f, target_frame=%s)",
        input_topic, legacy_input_topic, output_topic, goal_z,
        target_frame or "(none, pass-through)")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
