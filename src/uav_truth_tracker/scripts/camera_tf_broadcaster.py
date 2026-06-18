#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry


class CameraTfBroadcaster:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "mavros/local_position/odom")
        self.child_frame = rospy.get_param("~child_frame", "camera_link")
        self.parent_frame_override = rospy.get_param("~parent_frame", "")
        self.offset_x = rospy.get_param("~offset_x", 0.1)
        self.offset_y = rospy.get_param("~offset_y", 0.0)
        self.offset_z = rospy.get_param("~offset_z", 0.035)
        self.offset_roll = rospy.get_param("~offset_roll", 0.0)
        self.offset_pitch = rospy.get_param("~offset_pitch", 0.0)
        self.offset_yaw = rospy.get_param("~offset_yaw", 0.0)
        self.br = tf.TransformBroadcaster()
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=20)

        rospy.loginfo(
            "[CameraTF] %s -> %s from %s offset=(%.3f %.3f %.3f) rpy=(%.3f %.3f %.3f)",
            self.parent_frame_override or "odom.header.frame_id",
            self.child_frame,
            self.odom_topic,
            self.offset_x,
            self.offset_y,
            self.offset_z,
            self.offset_roll,
            self.offset_pitch,
            self.offset_yaw,
        )

    def odom_cb(self, msg):
        q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        q_offset = tf.transformations.quaternion_from_euler(
            self.offset_roll, self.offset_pitch, self.offset_yaw
        )
        q_child = tf.transformations.quaternion_multiply(q, q_offset)
        offset = tf.transformations.quaternion_matrix(q).dot(
            [self.offset_x, self.offset_y, self.offset_z, 1.0]
        )
        p = msg.pose.pose.position
        parent_frame = self.parent_frame_override or msg.header.frame_id or "map"
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

        self.br.sendTransform(
            (p.x + offset[0], p.y + offset[1], p.z + offset[2]),
            q_child,
            stamp,
            self.child_frame,
            parent_frame,
        )


if __name__ == "__main__":
    rospy.init_node("camera_tf_broadcaster")
    CameraTfBroadcaster()
    rospy.spin()
