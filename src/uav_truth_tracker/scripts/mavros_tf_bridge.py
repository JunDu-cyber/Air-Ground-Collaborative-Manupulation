#!/usr/bin/env python3
"""Subscribe to MAVROS local_position/pose and publish TF + drone visual marker"""

import os

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class MavrosTFBridge:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.marker_pub = rospy.Publisher(
            "/drone_visual", Marker, queue_size=10
        )
        self.path_pub = rospy.Publisher(
            "/drone_path", Path, queue_size=1
        )
        self.path = Path()
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.max_path_points = rospy.get_param("~max_path_points", 2000)
        self.publish_marker = rospy.get_param("~publish_marker", True)
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.pose_topic = rospy.get_param("~pose_topic", "/mavros/local_position/pose")
        if self.odom_topic:
            self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=20)
            rospy.logwarn("[MavrosTFBridge] publishing TF from odom %s", self.odom_topic)
        else:
            self.sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_cb, queue_size=20)
            rospy.logwarn("[MavrosTFBridge] publishing TF from pose %s", self.pose_topic)

        # 默认网格用 $PX4_DIR(或 ~/PX4-Autopilot)，不写死本机绝对路径——换台机器/队友 clone 也能用
        _px4 = os.environ.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot"))
        self.mesh_path = (
            "file://" + rospy.get_param("~mesh_path",
            os.path.join(_px4, "Tools/simulation/gazebo-classic/"
                         "sitl_gazebo-classic/models/iris/meshes/iris.stl"))
        )

    def odom_cb(self, msg: Odometry):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self.publish_pose(pose_msg)

    def pose_cb(self, msg: PoseStamped):
        self.publish_pose(msg)

    def publish_pose(self, msg: PoseStamped):
        frame_id = self.frame_id

        # TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = frame_id
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.br.sendTransform(t)

        if self.publish_marker:
            self.marker_pub.publish(self.make_mesh_marker(msg, frame_id))
            self.marker_pub.publish(self.make_delete_marker(1, msg.header.stamp, frame_id))
            self.marker_pub.publish(self.make_delete_marker(2, msg.header.stamp, frame_id))

            # Flight trace for RViz.
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = frame_id
            pose.pose = msg.pose
            self.path.header.stamp = msg.header.stamp
            self.path.header.frame_id = frame_id
            self.path.poses.append(pose)
            if len(self.path.poses) > self.max_path_points:
                self.path.poses = self.path.poses[-self.max_path_points:]
            self.path_pub.publish(self.path)

    def make_mesh_marker(self, msg: PoseStamped, frame_id: str):
        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = frame_id
        marker.ns = "drone"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = self.mesh_path
        marker.mesh_use_embedded_materials = False
        marker.pose = msg.pose
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.3
        marker.color.g = 0.3
        marker.color.b = 0.35
        marker.color.a = 1.0
        return marker

    def make_delete_marker(self, marker_id: int, stamp: rospy.Time, frame_id: str):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = "drone"
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker


if __name__ == "__main__":
    rospy.init_node("mavros_tf_bridge")
    MavrosTFBridge()
    rospy.spin()
