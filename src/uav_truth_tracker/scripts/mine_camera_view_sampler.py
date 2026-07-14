#!/usr/bin/env python3
"""Move the headless dataset camera between informative overhead viewpoints.

This is deliberately a dataset-generation utility, not a flight controller.  It
teleports the static camera in Gazebo so thousands of correctly labelled views
can be rendered quickly without spending most of the run on duplicate transit
frames.  The production UAV still collects with normal flight dynamics.
"""

import math
import random
import re

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import TransformStamped


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class MineCameraViewSampler:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "mine_camera_smoke")
        self.parent_frame = rospy.get_param("~parent_frame", "map")
        self.camera_frame = rospy.get_param("~camera_frame", "mine_camera_optical_frame")
        self.rate = max(float(rospy.get_param("~rate", 6.0)), 0.2)
        self.min_altitude = float(rospy.get_param("~min_altitude", 1.5))
        self.max_altitude = float(rospy.get_param("~max_altitude", 4.2))
        self.positive_view_ratio = float(rospy.get_param("~positive_view_ratio", 0.68))
        self.distractor_view_ratio = float(rospy.get_param("~distractor_view_ratio", 0.18))
        self.extent = float(rospy.get_param("~extent", 8.0))
        self.seed = int(rospy.get_param("~seed", 142))
        self.rng = random.Random(self.seed)
        self.mine_pattern = re.compile(r"^landmine(_.*)?$")
        self.distractor_pattern = re.compile(r"^distractor_disc_")
        self.mines = []
        self.distractors = []
        self.camera_pose = None
        self.last_tf_stamp = rospy.Time(0)
        self.pose_seq = -1
        self.broadcaster = tf2_ros.TransformBroadcaster()

        # The collector uses these parameters as a cross-process capture
        # barrier.  Frames are accepted only after Gazebo has rendered a stable
        # pose for the configured settle interval.
        rospy.set_param("/mine_dataset/camera_moving", True)
        rospy.set_param("/mine_dataset/camera_pose_seq", self.pose_seq)
        rospy.set_param("/mine_dataset/camera_last_move_time", -1.0)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._states_cb, queue_size=1)
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self._move)
        rospy.loginfo("[MineCameraSampler] model=%s rate=%.1fHz altitude=%.1f..%.1fm seed=%d",
                      self.model_name, self.rate, self.min_altitude, self.max_altitude, self.seed)

    def _states_cb(self, msg):
        self.mines = []
        self.distractors = []
        self.camera_pose = None
        for name, pose in zip(msg.name, msg.pose):
            if name == self.model_name:
                self.camera_pose = pose
            elif self.mine_pattern.match(name):
                self.mines.append((pose.position.x, pose.position.y))
            elif self.distractor_pattern.match(name):
                self.distractors.append((pose.position.x, pose.position.y))
        self._broadcast()

    def _broadcast(self):
        if self.camera_pose is None:
            return
        stamp = rospy.Time.now()
        # Gazebo can publish several ModelStates messages at one simulation
        # timestamp.  Re-publishing those floods tf2 during accelerated runs.
        if stamp <= self.last_tf_stamp:
            return
        self.last_tf_stamp = stamp
        p = self.camera_pose.position
        q = self.camera_pose.orientation
        wrapper_q = (q.x, q.y, q.z, q.w)
        # Optical axes used by mine_depth_camera: x=-body_y, y=-body_x, z=-body_z.
        optical_q = quat_multiply(wrapper_q, (math.sqrt(0.5), -math.sqrt(0.5), 0.0, 0.0))
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.camera_frame
        transform.transform.translation.x = p.x
        transform.transform.translation.y = p.y
        transform.transform.translation.z = p.z - 0.02
        transform.transform.rotation.x = optical_q[0]
        transform.transform.rotation.y = optical_q[1]
        transform.transform.rotation.z = optical_q[2]
        transform.transform.rotation.w = optical_q[3]
        self.broadcaster.sendTransform(transform)

    def _target(self, altitude):
        draw = self.rng.random()
        if draw < self.positive_view_ratio and self.mines:
            x, y = self.rng.choice(self.mines)
            # Offset the mine throughout the image instead of centering every target.
            return (x + self.rng.uniform(-0.34, 0.34) * altitude,
                    y + self.rng.uniform(-0.18, 0.18) * altitude)
        if draw < self.positive_view_ratio + self.distractor_view_ratio and self.distractors:
            x, y = self.rng.choice(self.distractors)
            return (x + self.rng.uniform(-0.25, 0.25) * altitude,
                    y + self.rng.uniform(-0.14, 0.14) * altitude)
        return (self.rng.uniform(-self.extent, self.extent),
                self.rng.uniform(-self.extent, self.extent))

    def _move(self, _event):
        if rospy.is_shutdown() or not self.mines:
            return
        altitude = self.rng.uniform(self.min_altitude, self.max_altitude)
        x, y = self._target(altitude)
        yaw = self.rng.uniform(-math.pi, math.pi)
        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = altitude
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)
        rospy.set_param("/mine_dataset/camera_moving", True)
        try:
            result = self.set_state(state)
            if not result.success:
                rospy.logwarn_throttle(3.0, "[MineCameraSampler] set state failed: %s", result.status_message)
                return
            self.pose_seq += 1
            rospy.set_param("/mine_dataset/camera_pose_seq", self.pose_seq)
            rospy.set_param("/mine_dataset/camera_last_move_time", rospy.Time.now().to_sec())
        except (rospy.ServiceException, rospy.ROSInterruptException) as exc:
            if not rospy.is_shutdown():
                rospy.logwarn_throttle(3.0, "[MineCameraSampler] service error: %s", exc)
        finally:
            rospy.set_param("/mine_dataset/camera_moving", False)


if __name__ == "__main__":
    rospy.init_node("mine_camera_view_sampler")
    MineCameraViewSampler()
    rospy.spin()
