#!/usr/bin/env python3
"""Relay Gazebo block-laser PointCloud output as PointCloud2."""

import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2


class PointCloud2Relay:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/uav0/velodyne_points_raw"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/uav0/velodyne_points"
        )
        self.frame_id = rospy.get_param("~frame_id", "")
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=2)
        rospy.Subscriber(self.input_topic, PointCloud, self.cloud_cb, queue_size=2)
        rospy.loginfo(
            "[PointCloud2Relay] %s PointCloud -> %s PointCloud2 frame_override='%s'",
            self.input_topic,
            self.output_topic,
            self.frame_id,
        )

    def cloud_cb(self, msg):
        header = msg.header
        if self.frame_id:
            header.frame_id = self.frame_id
        points = [(p.x, p.y, p.z) for p in msg.points]
        self.pub.publish(pc2.create_cloud_xyz32(header, points))


def main():
    rospy.init_node("pointcloud_to_pointcloud2_relay")
    PointCloud2Relay()
    rospy.spin()


if __name__ == "__main__":
    main()
