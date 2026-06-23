#!/usr/bin/env python3
"""Publish the UAV's pose-with-covariance in the UGV-centric (odom) frame.

Feeds elevation_mapping's ``robot_pose_with_covariance_topic`` (/uav/pose_cov) so
the repo's own uncertainty machinery propagates the UAV's localization error into
the egocentric elevation map. See config/elevation_mapping.yaml and
patches/perfect_sensor_processor_altitude.patch.

Frame / error chain (UGV static at init):

    W (map, GPS-anchored) --static--> O (odom, UGV egocentric)
    W --UAV nav--> U (uav_camera_link), with localization covariance Sigma_{W->U}

    Sigma_{O->U} = blkdiag(R,R) . Sigma_{W->U} . blkdiag(R,R)^T,  R = R_{odom<-map}

We publish Sigma_{O->U} (the UAV error expressed at the UGV centre) in the odom
frame. The sim UAV is a perfect static model, so the covariance is *injected*:
the values a real GPS+IMU-localized UAV would have. Three channels are consumed
by elevation_mapping:

  * att_rp / att_yaw -> per-point height variance, range-amplified by the lever
    arm (because config sets robot_base_frame_id = uav_camera_link).
  * alt  (z element) -> direct height variance (PerfectSensorProcessor patch).
  * gps_xy           -> height-on-slope, via min_horizontal_variance + fuse()
    (kept here too for completeness / future RobotMotionMapUpdater motion terms).

For a level UGV with isotropic horizontal and roll/pitch sigmas, Sigma_{O->U}
equals Sigma_{W->U} regardless of yaw; the R_{odom<-map} rotation only matters
once the UGV is tilted or the sigmas are anisotropic, and degrades gracefully to
identity if the map->odom transform is unavailable.
"""
import math

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def _rot_from_quat(q):
    """3x3 rotation matrix from a geometry_msgs Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def main():
    rospy.init_node('uav_pose_cov_publisher')

    map_frame = rospy.get_param('~map_frame', 'odom')            # == elevation map_frame_id
    base_frame = rospy.get_param('~base_frame', 'uav_camera_link')
    world_frame = rospy.get_param('~world_frame', 'map')         # inertial / GPS frame
    rate_hz = rospy.get_param('~rate', 10.0)

    # Covariance source:
    #   injected    -> synthetic diagonal sigmas below (mock UAV / testing)
    #   mavros_odom -> the real UAV pose covariance from a nav_msgs/Odometry
    #                  topic (e.g. /mavros/local_position/odom, PX4 EKF2).
    cov_source = rospy.get_param('~cov_source', 'injected')
    odom_topic = rospy.get_param('~odom_topic', '/mavros/local_position/odom')

    # Injected UAV localization sigmas (see module docstring).
    gps_xy = rospy.get_param('~gps_xy_sigma', 0.5)               # [m]  horizontal GPS
    alt = rospy.get_param('~alt_sigma', 0.3)                     # [m]  altitude
    att_rp = math.radians(rospy.get_param('~att_rp_sigma_deg', 0.5))   # roll/pitch
    att_yaw = math.radians(rospy.get_param('~att_yaw_sigma_deg', 1.0))  # yaw

    # World-frame diagonal covariance, ROS order [x, y, z, roll, pitch, yaw].
    sigma_injected = np.diag([gps_xy ** 2, gps_xy ** 2, alt ** 2,
                              att_rp ** 2, att_rp ** 2, att_yaw ** 2])

    # mavros_odom: cache the latest real 6x6 pose covariance (world frame).
    cov_state = {'mavros': None}

    def _odom_cb(msg):
        cov_state['mavros'] = np.array(msg.pose.covariance,
                                       dtype=float).reshape(6, 6)

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    if cov_source == 'mavros_odom':
        rospy.Subscriber(odom_topic, Odometry, _odom_cb, queue_size=10)
    pub = rospy.Publisher('/uav/pose_cov', PoseWithCovarianceStamped, queue_size=1)
    rate = rospy.Rate(rate_hz)

    if cov_source == 'mavros_odom':
        rospy.loginfo('[uav_pose_cov] publishing Sigma_{%s->%s} on /uav/pose_cov '
                      '(cov_source=mavros_odom from %s)',
                      map_frame, base_frame, odom_topic)
    else:
        rospy.loginfo('[uav_pose_cov] publishing Sigma_{%s->%s} on /uav/pose_cov '
                      '(cov_source=injected gps_xy=%.2fm alt=%.2fm '
                      'att_rp=%.2fdeg att_yaw=%.2fdeg)',
                      map_frame, base_frame, gps_xy, alt,
                      math.degrees(att_rp), math.degrees(att_yaw))

    while not rospy.is_shutdown():
        # Pose of the UAV expressed in the egocentric (odom) frame.
        try:
            tf_ou = tf_buffer.lookup_transform(map_frame, base_frame,
                                               rospy.Time(0), rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(5.0, '[uav_pose_cov] waiting for TF %s->%s: %s',
                                   map_frame, base_frame, exc)
            rate.sleep()
            continue

        # Pick the world-frame covariance (injected diagonal, or the real one
        # cached from MAVROS odometry).
        if cov_source == 'mavros_odom':
            sigma_w = cov_state['mavros']
            if sigma_w is None:
                rospy.logwarn_throttle(5.0, '[uav_pose_cov] waiting for %s ...',
                                       odom_topic)
                rate.sleep()
                continue
        else:
            sigma_w = sigma_injected

        # Re-express the world-frame covariance in the odom frame (R_{odom<-map}).
        sigma = sigma_w
        try:
            tf_mw = tf_buffer.lookup_transform(map_frame, world_frame,
                                               rospy.Time(0), rospy.Duration(0.05))
            R = _rot_from_quat(tf_mw.transform.rotation)
            Rb = np.zeros((6, 6))
            Rb[:3, :3] = R
            Rb[3:, 3:] = R
            sigma = Rb @ sigma_w @ Rb.T
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass  # identity fallback (exact for a level UGV + isotropic sigmas)

        msg = PoseWithCovarianceStamped()
        stamp = tf_ou.header.stamp
        msg.header.stamp = stamp if not stamp.is_zero() else rospy.Time.now()
        msg.header.frame_id = map_frame
        msg.pose.pose.position.x = tf_ou.transform.translation.x
        msg.pose.pose.position.y = tf_ou.transform.translation.y
        msg.pose.pose.position.z = tf_ou.transform.translation.z
        msg.pose.pose.orientation = tf_ou.transform.rotation
        msg.pose.covariance = sigma.flatten().tolist()
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
