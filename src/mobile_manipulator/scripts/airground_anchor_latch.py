#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Capture-once-then-latch the egocentric air-ground anchor: odom -> uav0/map_local.

In the full-egocentric stack the UGV's `odom` (DLIO) is the only authoritative
frame and there is no `map`. But the UAV's cloud lives under its MAVROS-local
origin `uav0/map_local`, which must be tied to the UGV `odom` so the aerial cloud
lands in the elevation map. This node establishes that tie ONCE, during the UAV
mapping epoch, using a global snapshot the UGV is permitted to read only then:

  * the UGV's TRUE world pose  T_world_base  (from /ground_truth/state; full pose,
    so the anchor yaw is observable -- a single GPS fix would leave yaw ambiguous)
  * the UGV's odom pose        T_odom_base   (from /state_estimation, DLIO)

  => world->odom :  T_world_odom = T_world_base * inv(T_odom_base)
  The UAV MAVROS-local origin sits at a known offset in the world (its spawn XY,
  ground Z), identity rotation:  T_world_uavlocal = trans(uav_spawn), I.
  => odom->uav0/map_local :  T_odom_uavlocal = inv(T_world_odom) * T_world_uavlocal

After averaging N stable samples it LATCHES that transform and republishes it as a
fixed static TF indefinitely; the global snapshot inputs are then dropped, so the
UGV drives autonomously on LIO-only odometry with no further GPS/ground-truth use.
Re-trigger with the ~/relatch service if you need to re-anchor.
"""
import threading

import numpy as np
import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse


def _mat_from_odom(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    T[:3, 3] = [p.x, p.y, p.z]
    return T


class AnchorLatch:
    def __init__(self):
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.uav_map_frame = rospy.get_param('~uav_map_frame', 'uav0/map_local')
        self.truth_topic = rospy.get_param('~ugv_truth_topic', '/ground_truth/state')
        self.odom_topic = rospy.get_param('~ugv_odom_topic', '/state_estimation')
        # UAV MAVROS-local origin in the world (== old map->uav0/map_local offset)
        self.uav_spawn = [rospy.get_param('~uav_spawn_x', 0.0),
                          rospy.get_param('~uav_spawn_y', -18.0),
                          rospy.get_param('~uav_spawn_z', 0.0)]
        self.n_samples = int(rospy.get_param('~num_samples', 30))
        # Only sample while the UGV is stationary (skip the spawn-settling
        # transient, where ground-truth and the still-converging LIO disagree and
        # the robot is physically moving). The co-sim mapping epoch is stationary,
        # so this just defers the latch until the platform is at rest.
        self.max_lin_vel = float(rospy.get_param('~max_lin_vel', 0.03))   # m/s
        self.max_ang_vel = float(rospy.get_param('~max_ang_vel', 0.03))   # rad/s

        self.truth = None
        self.odom = None
        self.samples = []
        self.latched_T = None
        self.lock = threading.Lock()

        self.br = tf2_ros.StaticTransformBroadcaster()
        rospy.Subscriber(self.truth_topic, Odometry, self._truth_cb, queue_size=20)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=20)
        rospy.Service('~relatch', Empty, self._relatch)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._tick)
        rospy.loginfo('[anchor] capturing %s + %s -> latch %s->%s after %d samples',
                      self.truth_topic, self.odom_topic, self.odom_frame,
                      self.uav_map_frame, self.n_samples)

    def _truth_cb(self, msg):
        self.truth = msg

    def _odom_cb(self, msg):
        self.odom = msg

    def _relatch(self, _req):
        with self.lock:
            self.samples = []
            self.latched_T = None
        rospy.logwarn('[anchor] re-latch requested; recapturing')
        return EmptyResponse()

    def _tick(self, _evt):
        with self.lock:
            if self.latched_T is not None:
                self._publish(self.latched_T)
                return
            if self.truth is None or self.odom is None:
                return
            # gate on stationarity using the (clean) ground-truth velocity; the
            # LIO odom twist is too noisy at standstill to gate on.
            if self._moving(self.truth):
                if self.samples:
                    rospy.loginfo('[anchor] motion detected; resetting sample buffer')
                self.samples = []
                return
            # world->odom = T_world_base * inv(T_odom_base)
            T_world_base = _mat_from_odom(self.truth)
            T_odom_base = _mat_from_odom(self.odom)
            T_world_odom = T_world_base.dot(tft.inverse_matrix(T_odom_base))
            self.samples.append(T_world_odom)
            if len(self.samples) >= self.n_samples:
                T_world_odom_avg = self._average(self.samples)
                T_world_uavlocal = tft.translation_matrix(self.uav_spawn)
                T_odom_uavlocal = tft.inverse_matrix(T_world_odom_avg).dot(T_world_uavlocal)
                self.latched_T = T_odom_uavlocal
                t = T_odom_uavlocal[:3, 3]
                yaw = tft.euler_from_matrix(T_odom_uavlocal)[2]
                rospy.logwarn('[anchor] LATCHED %s->%s  t=(%.3f, %.3f, %.3f) yaw=%.3f rad '
                              '(GPS/ground-truth now dropped)', self.odom_frame,
                              self.uav_map_frame, t[0], t[1], t[2], yaw)

    def _moving(self, odom):
        v = odom.twist.twist.linear
        w = odom.twist.twist.angular
        lin = (v.x * v.x + v.y * v.y + v.z * v.z) ** 0.5
        ang = (w.x * w.x + w.y * w.y + w.z * w.z) ** 0.5
        return lin > self.max_lin_vel or ang > self.max_ang_vel

    @staticmethod
    def _average(mats):
        """Average a set of homogeneous transforms (mean translation, quaternion mean)."""
        ts = np.array([M[:3, 3] for M in mats])
        qs = np.array([tft.quaternion_from_matrix(M) for M in mats])
        # sign-align quaternions before averaging
        ref = qs[0]
        for i in range(len(qs)):
            if np.dot(qs[i], ref) < 0:
                qs[i] = -qs[i]
        q_mean = qs.mean(axis=0)
        q_mean /= np.linalg.norm(q_mean)
        T = tft.quaternion_matrix(q_mean)
        T[:3, 3] = ts.mean(axis=0)
        return T

    def _publish(self, T):
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.uav_map_frame
        msg.transform.translation.x = T[0, 3]
        msg.transform.translation.y = T[1, 3]
        msg.transform.translation.z = T[2, 3]
        q = tft.quaternion_from_matrix(T)
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        self.br.sendTransform(msg)


def main():
    rospy.init_node('airground_anchor_latch')
    AnchorLatch()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
