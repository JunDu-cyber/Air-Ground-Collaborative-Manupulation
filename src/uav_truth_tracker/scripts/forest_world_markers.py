#!/usr/bin/env python3
"""Publish lightweight RViz markers for the forest corridor Gazebo world."""

import math
import os
import xml.etree.ElementTree as ET

import rospy
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


def _float_list(text, default):
    if not text:
        return default
    values = [float(v) for v in text.split()]
    return values + default[len(values):]


def _set_pose(marker, pose):
    x, y, z, roll, pitch, yaw = pose
    q = quaternion_from_euler(roll, pitch, yaw)
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]


def _set_color(marker, rgba):
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba


class ForestWorldMarkers:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.topic = rospy.get_param("~topic", "/forest_world_markers")
        self.world_file = rospy.get_param(
            "~world_file", os.path.expanduser("~/worlds/forest_corridor.world")
        )
        self.publish_rate = rospy.get_param("~publish_rate", 1.0)

        self.pub = rospy.Publisher(self.topic, MarkerArray, queue_size=1, latch=True)
        self.markers = self._load_markers()
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._publish)
        rospy.loginfo(
            "[ForestWorldMarkers] publishing %d markers from %s on %s",
            len(self.markers.markers),
            self.world_file,
            self.topic,
        )

    def _new_marker(self, marker_id, ns, marker_type, pose, scale, color):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        _set_pose(marker, pose)
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        _set_color(marker, color)
        return marker

    def _add_ground(self, markers, marker_id):
        markers.append(
            self._new_marker(
                marker_id,
                "forest_ground",
                Marker.CUBE,
                (0.0, 0.0, -0.03, 0.0, 0.0, 0.0),
                (14.0, 45.0, 0.03),
                (0.12, 0.18, 0.12, 0.35),
            )
        )
        return marker_id + 1

    def _add_tree(self, markers, marker_id, pose):
        x, y, z, roll, pitch, yaw = pose
        markers.append(
            self._new_marker(
                marker_id,
                "forest_trees",
                Marker.CYLINDER,
                (x, y, z + 1.05, roll, pitch, yaw),
                (0.34, 0.34, 2.10),
                (0.30, 0.17, 0.08, 1.0),
            )
        )
        marker_id += 1
        markers.append(
            self._new_marker(
                marker_id,
                "forest_canopy",
                Marker.SPHERE,
                (x, y, z + 2.45, roll, pitch, yaw),
                (1.6, 1.6, 1.4),
                (0.10, 0.35, 0.13, 0.78),
            )
        )
        return marker_id + 1

    def _add_collision_marker(self, markers, marker_id, model_pose, link):
        link_pose = _float_list(link.findtext("pose"), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        pose = [
            model_pose[0] + link_pose[0],
            model_pose[1] + link_pose[1],
            model_pose[2] + link_pose[2],
            model_pose[3] + link_pose[3],
            model_pose[4] + link_pose[4],
            model_pose[5] + link_pose[5],
        ]

        collision = link.find("collision")
        if collision is None:
            return marker_id
        geometry = collision.find("geometry")
        if geometry is None:
            return marker_id

        box = geometry.find("box")
        if box is not None:
            size = _float_list(box.findtext("size"), [1.0, 1.0, 1.0])
            markers.append(
                self._new_marker(
                    marker_id,
                    "forest_obstacles",
                    Marker.CUBE,
                    pose,
                    tuple(size[:3]),
                    (0.55, 0.42, 0.28, 0.9),
                )
            )
            return marker_id + 1

        cylinder = geometry.find("cylinder")
        if cylinder is not None:
            radius = float(cylinder.findtext("radius", "0.2"))
            length = float(cylinder.findtext("length", "1.0"))
            markers.append(
                self._new_marker(
                    marker_id,
                    "forest_obstacles",
                    Marker.CYLINDER,
                    pose,
                    (2.0 * radius, 2.0 * radius, length),
                    (0.42, 0.26, 0.15, 0.92),
                )
            )
            return marker_id + 1

        return marker_id

    def _load_markers(self):
        marker_array = MarkerArray()
        markers = []
        marker_id = self._add_ground(markers, 0)

        tree = ET.parse(self.world_file)
        world = tree.getroot().find("world")
        if world is None:
            raise RuntimeError("No <world> element in %s" % self.world_file)

        for include in world.findall("include"):
            uri = include.findtext("uri", "")
            pose = _float_list(include.findtext("pose"), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            if uri == "model://ego_tree":
                marker_id = self._add_tree(markers, marker_id, pose)

        for model in world.findall("model"):
            model_pose = _float_list(model.findtext("pose"), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            for link in model.findall("link"):
                marker_id = self._add_collision_marker(markers, marker_id, model_pose, link)

        marker_array.markers = markers
        return marker_array

    def _publish(self, _event):
        now = rospy.Time.now()
        for marker in self.markers.markers:
            marker.header.stamp = now
        self.pub.publish(self.markers)


if __name__ == "__main__":
    rospy.init_node("forest_world_markers")
    ForestWorldMarkers()
    rospy.spin()
