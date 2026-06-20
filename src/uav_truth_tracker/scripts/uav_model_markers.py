#!/usr/bin/env python3

import copy

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from px4_paths import default_iris_mesh_resource


class UavModelMarkers:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.mesh_resource = rospy.get_param(
            "~mesh_resource", default_iris_mesh_resource()
        )
        self.scale = rospy.get_param("~scale", 1.0)
        self.publish_rate = rospy.get_param("~publish_rate", 10.0)

        self.uavs = [
            {
                "name": "uav0",
                "topic": rospy.get_param("~uav0_odom", "/uav0/mavros/local_position/odom"),
                "offset": (
                    rospy.get_param("~uav0_offset_x", 0.0),
                    rospy.get_param("~uav0_offset_y", 0.0),
                    rospy.get_param("~uav0_offset_z", 0.0),
                ),
                "color": (0.1, 0.45, 1.0, 1.0),
            },
            {
                "name": "uav1",
                "topic": rospy.get_param("~uav1_odom", "/uav1/mavros/local_position/odom"),
                "offset": (
                    rospy.get_param("~uav1_offset_x", 2.0),
                    rospy.get_param("~uav1_offset_y", 0.0),
                    rospy.get_param("~uav1_offset_z", 0.0),
                ),
                "color": (1.0, 0.35, 0.1, 1.0),
            },
        ]

        self.latest = {}
        for uav in self.uavs:
            rospy.Subscriber(
                uav["topic"],
                Odometry,
                self.odom_cb,
                callback_args=uav["name"],
                queue_size=20,
            )

        self.pub = rospy.Publisher("uav_models/markers", MarkerArray, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish)
        rospy.loginfo("[UavModelMarkers] publishing /uav_models/markers in frame %s", self.frame_id)

    def odom_cb(self, msg, name):
        self.latest[name] = msg

    def publish(self, _event):
        markers = MarkerArray()
        now = rospy.Time.now()
        marker_id = 0

        for uav in self.uavs:
            odom = self.latest.get(uav["name"])
            if odom is None:
                continue

            mesh = Marker()
            mesh.header.frame_id = self.frame_id
            mesh.header.stamp = now
            mesh.ns = "uav_models"
            mesh.id = marker_id
            marker_id += 1
            mesh.type = Marker.MESH_RESOURCE
            mesh.action = Marker.ADD
            mesh.mesh_resource = self.mesh_resource
            mesh.mesh_use_embedded_materials = False
            mesh.scale.x = self.scale
            mesh.scale.y = self.scale
            mesh.scale.z = self.scale
            mesh.color.r, mesh.color.g, mesh.color.b, mesh.color.a = uav["color"]

            mesh.pose = copy.deepcopy(odom.pose.pose)
            mesh.pose.position.x += uav["offset"][0]
            mesh.pose.position.y += uav["offset"][1]
            mesh.pose.position.z += uav["offset"][2]
            markers.markers.append(mesh)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = now
            label.ns = "uav_labels"
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = copy.deepcopy(mesh.pose)
            label.pose.position.z += 0.45
            label.scale.z = 0.22
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = uav["name"]
            markers.markers.append(label)

        if markers.markers:
            self.pub.publish(markers)


if __name__ == "__main__":
    rospy.init_node("uav_model_markers")
    UavModelMarkers()
    rospy.spin()
