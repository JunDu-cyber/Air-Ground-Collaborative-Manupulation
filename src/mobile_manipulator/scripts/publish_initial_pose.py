#!/usr/bin/env python3
"""Publish initial pose to AMCL so map->odom TF appears without needing RViz interaction."""
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def main():
    rospy.init_node('initial_pose_publisher', anonymous=True)

    x   = rospy.get_param('~x',   1.5)
    y   = rospy.get_param('~y',   0.5)
    yaw = rospy.get_param('~yaw', 0.0)

    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)

    import math
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp    = rospy.Time.now()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    # covariance diagonal: xx, yy, aa
    msg.pose.covariance[0]  = 0.25   # x
    msg.pose.covariance[7]  = 0.25   # y
    msg.pose.covariance[35] = 0.07   # yaw

    # Wait for AMCL to come up, then publish once
    rospy.sleep(2.0)
    pub.publish(msg)
    rospy.loginfo(f'Published initial pose: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')
    rospy.sleep(1.0)  # Keep latched long enough for AMCL to receive it

if __name__ == '__main__':
    main()
