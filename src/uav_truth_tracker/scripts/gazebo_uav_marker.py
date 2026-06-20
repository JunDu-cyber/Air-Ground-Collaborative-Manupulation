#!/usr/bin/env python3
"""Publish RViz Marker from Gazebo model pose — no MAVROS required.

Subscribes to /gazebo/model_states to track the UAV (and optionally UGV)
actual pose in Gazebo, and publishes Marker messages so RViz can show
the 3D mesh at the correct live position.
"""

import rospy
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from px4_paths import default_iris_mesh_resource


class GazeboUavMarker:
    def __init__(self):
        # --- UAV config ---
        self.uav_model_name = rospy.get_param(
            "~uav_model_name", "iris_depth_camera"
        )
        self.uav_mesh = rospy.get_param(
            "~uav_mesh_resource", default_iris_mesh_resource()
        )
        self.uav_ns = rospy.get_param("~uav_ns", "drone")
        self.uav_marker_topic = rospy.get_param(
            "~uav_marker_topic", "/drone_visual"
        )
        self.uav_path_topic = rospy.get_param("~uav_path_topic", "/drone_path")
        self.uav_scale = rospy.get_param("~uav_scale", 1.0)
        self.uav_color = (
            rospy.get_param("~uav_color_r", 0.1),
            rospy.get_param("~uav_color_g", 0.45),
            rospy.get_param("~uav_color_b", 1.0),
            rospy.get_param("~uav_color_a", 1.0),
        )

        # --- UGV config (optional, set ugv_model_name to empty to disable) ---
        self.ugv_model_name = rospy.get_param("~ugv_model_name", "")
        self.ugv_mesh = rospy.get_param(
            "~ugv_mesh_resource",
            "package://husky_description/meshes/base_link.dae",
        )
        self.ugv_ns = rospy.get_param("~ugv_ns", "ugv")
        self.ugv_marker_topic = rospy.get_param(
            "~ugv_marker_topic", "/ugv_visual"
        )
        self.ugv_path_topic = rospy.get_param("~ugv_path_topic", "/ugv_path")
        self.ugv_scale = rospy.get_param("~ugv_scale", 1.0)
        self.ugv_color = (
            rospy.get_param("~ugv_color_r", 0.1),
            rospy.get_param("~ugv_color_g", 0.55),
            rospy.get_param("~ugv_color_b", 0.25),
            rospy.get_param("~ugv_color_a", 1.0),
        )

        # --- Static fallback pose (used until first model_states msg) ---
        self.fallback_x = rospy.get_param("~fallback_x", 0.0)
        self.fallback_y = rospy.get_param("~fallback_y", -18.0)
        self.fallback_z = rospy.get_param("~fallback_z", 1.5)
        self.fallback_yaw = rospy.get_param("~fallback_yaw", 1.5707963)

        # --- State ---
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = rospy.get_param("~publish_rate", 10.0)

        self._last_uav_pose = None
        self._last_ugv_pose = None

        # Fallback pose
        q = quaternion_from_euler(0.0, 0.0, self.fallback_yaw)
        fallback = PoseStamped()
        fallback.header.frame_id = self.frame_id
        fallback.pose.position.x = self.fallback_x
        fallback.pose.position.y = self.fallback_y
        fallback.pose.position.z = self.fallback_z
        fallback.pose.orientation.x = q[0]
        fallback.pose.orientation.y = q[1]
        fallback.pose.orientation.z = q[2]
        fallback.pose.orientation.w = q[3]
        self._last_uav_pose = fallback
        self._last_ugv_pose = fallback  # same default

        # --- Publishers ---
        self.uav_marker_pub = rospy.Publisher(
            self.uav_marker_topic, Marker, queue_size=1, latch=True
        )
        self.uav_path_pub = rospy.Publisher(
            self.uav_path_topic, Path, queue_size=1, latch=True
        )
        self.ugv_marker_pub = None
        self.ugv_path_pub = None
        if self.ugv_model_name:
            self.ugv_marker_pub = rospy.Publisher(
                self.ugv_marker_topic, Marker, queue_size=1, latch=True
            )
            self.ugv_path_pub = rospy.Publisher(
                self.ugv_path_topic, Path, queue_size=1, latch=True
            )

        # --- Subscriber ---
        rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._model_states_cb, queue_size=10
        )

        # --- Timer ---
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self._publish
        )

        rospy.loginfo(
            "[GazeboUavMarker] tracking Gazebo model '%s' on %s (frame %s)",
            self.uav_model_name,
            self.uav_marker_topic,
            self.frame_id,
        )
        if self.ugv_model_name:
            rospy.loginfo(
                "[GazeboUavMarker] tracking Gazebo model '%s' on %s",
                self.ugv_model_name,
                self.ugv_marker_topic,
            )

    def _model_states_cb(self, msg):
        stamp = rospy.Time.now()
        for name, pose in zip(msg.name, msg.pose):
            if name == self.uav_model_name:
                ps = PoseStamped()
                ps.header.frame_id = self.frame_id
                ps.header.stamp = stamp
                ps.pose = pose
                self._last_uav_pose = ps

            if self.ugv_model_name and name == self.ugv_model_name:
                ps = PoseStamped()
                ps.header.frame_id = self.frame_id
                ps.header.stamp = stamp
                ps.pose = pose
                self._last_ugv_pose = ps

    @staticmethod
    def _make_marker(ns, mid, mesh, pose, scale, color, frame_id):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = mesh
        m.mesh_use_embedded_materials = False
        m.pose = pose.pose
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    @staticmethod
    def _make_delete_marker(ns, mid, frame_id):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.action = Marker.DELETE
        return m

    @staticmethod
    def _make_path(ns, pose_stamped, frame_id):
        p = Path()
        p.header.frame_id = frame_id
        p.header.stamp = pose_stamped.header.stamp
        ps = PoseStamped()
        ps.header = pose_stamped.header
        ps.pose = pose_stamped.pose
        p.poses = [ps]
        return p

    def _publish(self, _event):
        # UAV
        if self._last_uav_pose is not None:
            m = self._make_marker(
                self.uav_ns, 0, self.uav_mesh,
                self._last_uav_pose, self.uav_scale, self.uav_color,
                self.frame_id,
            )
            self.uav_marker_pub.publish(m)
            self.uav_marker_pub.publish(
                self._make_delete_marker(self.uav_ns, 1, self.frame_id)
            )
            self.uav_marker_pub.publish(
                self._make_delete_marker(self.uav_ns, 2, self.frame_id)
            )
            self.uav_path_pub.publish(
                self._make_path(self.uav_ns, self._last_uav_pose, self.frame_id)
            )

        # UGV
        if self.ugv_marker_pub is not None and self._last_ugv_pose is not None:
            m = self._make_marker(
                self.ugv_ns, 0, self.ugv_mesh,
                self._last_ugv_pose, self.ugv_scale, self.ugv_color,
                self.frame_id,
            )
            self.ugv_marker_pub.publish(m)
            self.ugv_path_pub.publish(
                self._make_path(self.ugv_ns, self._last_ugv_pose, self.frame_id)
            )


if __name__ == "__main__":
    rospy.init_node("gazebo_uav_marker")
    GazeboUavMarker()
    rospy.spin()
