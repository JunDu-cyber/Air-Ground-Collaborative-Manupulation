#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from px4_paths import default_iris_mesh_resource


def main():
    rospy.init_node("static_uav_marker")

    frame_id = rospy.get_param("~frame_id", "map")
    marker_ns = rospy.get_param("~ns", "drone")
    marker_topic = rospy.get_param("~marker_topic", "/drone_visual")
    path_topic = rospy.get_param("~path_topic", "/drone_path")
    mesh_resource = rospy.get_param(
        "~mesh_resource", default_iris_mesh_resource()
    )
    if "://" not in mesh_resource:
        mesh_resource = "file://" + mesh_resource
    x = rospy.get_param("~x", 0.0)
    y = rospy.get_param("~y", -18.0)
    z = rospy.get_param("~z", 1.5)
    yaw = rospy.get_param("~yaw", 1.5707963)
    scale = rospy.get_param("~scale", 1.0)
    color_r = rospy.get_param("~color_r", 0.3)
    color_g = rospy.get_param("~color_g", 0.3)
    color_b = rospy.get_param("~color_b", 0.35)
    color_a = rospy.get_param("~color_a", 1.0)
    rate_hz = rospy.get_param("~rate", 5.0)

    q = quaternion_from_euler(0.0, 0.0, yaw)
    marker_pub = rospy.Publisher(marker_topic, Marker, queue_size=1, latch=True)
    path_pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = marker_ns
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.mesh_resource = mesh_resource
    marker.mesh_use_embedded_materials = False
    marker.pose = pose.pose
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = color_r
    marker.color.g = color_g
    marker.color.b = color_b
    marker.color.a = color_a

    path = Path()
    path.header.frame_id = frame_id
    path.poses = [pose]

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        pose.header.stamp = now
        marker.header.stamp = now
        path.header.stamp = now
        path.poses[0].header.stamp = now
        marker_pub.publish(marker)
        path_pub.publish(path)
        rate.sleep()


if __name__ == "__main__":
    main()
