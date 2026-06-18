#!/usr/bin/env python3
"""Gazebo Odom Bridge — replaces MAVROS for odometry in simulation.

Subscribes to /gazebo/model_states to track the UAV pose in real time,
then publishes the topics that MAVROS would normally provide:

  /mavros/local_position/odom     (nav_msgs/Odometry)
  /mavros/local_position/pose     (geometry_msgs/PoseStamped)
  /mavros/imu/data                (sensor_msgs/Imu)

Also broadcasts TF  map → base_link  dynamically and publishes
/drone_visual / /drone_path markers so RViz shows the 3D model.

This lets EGO-Planner, OctoMap, LiDAR relay, and UAV→UGV point cloud
all work WITHOUT installing ros-noetic-mavros.
"""

import copy
import math

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from geometry_msgs.msg import Vector3, Quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


class GazeboOdomBridge:
    def __init__(self):
        # --- Which Gazebo model to track ---
        self.model_name = rospy.get_param(
            "~model_name", "iris_depth_camera"
        )

        # --- Frame configuration ---
        self.world_frame = rospy.get_param("~world_frame", "map")
        self.body_frame = rospy.get_param("~body_frame", "base_link")
        self.publish_rate = rospy.get_param("~publish_rate", 30.0)

        # --- Marker mesh ---
        self.mesh_resource = rospy.get_param(
            "~mesh_resource",
            "file:///home/lnwuu/PX4-Autopilot/Tools/simulation/gazebo-classic/"
            "sitl_gazebo-classic/models/iris/meshes/iris.stl",
        )

        # --- State ---
        self._last_pose = None   # PoseStamped
        self._last_twist = None  # TwistStamped
        self._model_found = False
        self._path = Path()
        self._max_path_points = rospy.get_param("~max_path_points", 5000)

        # --- Publishers ---
        self.odom_pub = rospy.Publisher(
            "/mavros/local_position/odom", Odometry, queue_size=10
        )
        self.pose_pub = rospy.Publisher(
            "/mavros/local_position/pose", PoseStamped, queue_size=10
        )
        self.imu_pub = rospy.Publisher(
            "/mavros/imu/data", Imu, queue_size=10
        )
        self.visual_pub = rospy.Publisher(
            "/drone_visual", Marker, queue_size=1, latch=True
        )
        self.path_pub = rospy.Publisher(
            "/drone_path", Path, queue_size=1, latch=True
        )

        # --- TF broadcaster ---
        self.tf_br = tf2_ros.TransformBroadcaster()

        # --- Subscriber ---
        rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._cb, queue_size=10
        )

        # --- Timer ---
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_rate, 1.0)), self._publish
        )

        rospy.loginfo(
            "[GazeboOdomBridge] tracking model '%s', "
            "publishing /mavros/local_position/odom + TF %s→%s @ %.1f Hz",
            self.model_name, self.world_frame, self.body_frame,
            self.publish_rate,
        )
        rospy.loginfo(
            "[GazeboOdomBridge] install ros-noetic-mavros for full flight; "
            "this bridge provides odometry for mapping & planning only."
        )

    # ── Gazebo callback ───────────────────────────────────────────────

    def _cb(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            if self._model_found:
                rospy.logwarn_throttle(
                    5.0,
                    "[GazeboOdomBridge] model '%s' disappeared from "
                    "/gazebo/model_states", self.model_name,
                )
                self._model_found = False
            return

        if not self._model_found:
            rospy.loginfo(
                "[GazeboOdomBridge] found model '%s' in /gazebo/model_states",
                self.model_name,
            )
            self._model_found = True

        pose = msg.pose[idx]
        twist = msg.twist[idx]

        stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()

        ps = PoseStamped()
        ps.header.frame_id = self.world_frame
        ps.header.stamp = stamp
        ps.pose = pose
        self._last_pose = ps

        ts = TwistStamped()
        ts.header.frame_id = self.body_frame
        ts.header.stamp = stamp
        ts.twist = twist
        self._last_twist = ts

    # ── Timer publish ─────────────────────────────────────────────────

    def _publish(self, _event):
        if self._last_pose is None:
            return

        now = self._last_pose.header.stamp
        pose = self._last_pose.pose
        twist = self._last_twist.twist if self._last_twist else None

        # ── TF  map → base_link ──
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.body_frame
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.tf_br.sendTransform(t)

        # ── /mavros/local_position/pose ──
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = self.world_frame
        ps.pose = pose
        self.pose_pub.publish(ps)

        # ── /mavros/local_position/odom ──
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.body_frame
        odom.pose.pose = pose
        odom.pose.covariance[0] = 0.01   # small position uncertainty
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 0.01
        odom.pose.covariance[21] = 0.001  # small orientation uncertainty
        odom.pose.covariance[28] = 0.001
        odom.pose.covariance[35] = 0.001
        if twist is not None:
            odom.twist.twist = twist
        self.odom_pub.publish(odom)

        # ── /mavros/imu/data ──
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.body_frame
        imu.orientation = pose.orientation
        if twist is not None:
            imu.angular_velocity = twist.angular
            # crude linear acceleration from velocity delta
            imu.linear_acceleration = Vector3(0.0, 0.0, 0.0)
        # covariance not set → -1 (unknown)
        self.imu_pub.publish(imu)

        # ── /drone_visual Marker ──
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = self.world_frame
        marker.ns = "drone"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = self.mesh_resource
        marker.mesh_use_embedded_materials = False
        marker.pose = pose
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.visual_pub.publish(marker)

        # ── /drone_path ──
        path_pose = PoseStamped()
        path_pose.header.stamp = now
        path_pose.header.frame_id = self.world_frame
        path_pose.pose = pose
        self._path.header.stamp = now
        self._path.header.frame_id = self.world_frame
        self._path.poses.append(path_pose)
        if len(self._path.poses) > self._max_path_points:
            self._path.poses = self._path.poses[-self._max_path_points:]
        self.path_pub.publish(self._path)


if __name__ == "__main__":
    rospy.init_node("gazebo_odom_bridge")
    GazeboOdomBridge()
    rospy.spin()
