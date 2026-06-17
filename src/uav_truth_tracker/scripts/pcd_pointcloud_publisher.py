#!/usr/bin/env python3
"""Publish an ASCII PCD file as a latched PointCloud2 topic."""

import os

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class PcdPointCloudPublisher(object):
    def __init__(self):
        self.pcd_file = os.path.expanduser(
            rospy.get_param(
                "~pcd_file", "~/pointcloud_maps/uav_points_map_latest.pcd"
            )
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/uav0/mapping/points_saved"
        )
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 1.0))
        self.latch = bool(rospy.get_param("~latch", True))

        self.points = self.load_ascii_pcd(self.pcd_file)
        self.pub = rospy.Publisher(
            self.output_topic, PointCloud2, queue_size=1, latch=self.latch
        )

        rospy.loginfo(
            "[PcdPointCloudPublisher] file=%s points=%d output=%s frame=%s rate=%.2f latch=%s",
            self.pcd_file,
            len(self.points),
            self.output_topic,
            self.frame_id,
            self.publish_rate,
            self.latch,
        )

    def spin(self):
        self.publish_once()
        if self.publish_rate <= 0.0:
            rospy.spin()
            return
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.publish_once()
            rate.sleep()

    def publish_once(self):
        header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        self.pub.publish(pc2.create_cloud_xyz32(header, self.points))

    def load_ascii_pcd(self, path):
        if not os.path.exists(path):
            raise FileNotFoundError(path)

        fields = []
        data_mode = None
        points = []
        with open(path, "r") as f:
            for raw_line in f:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                upper = line.upper()
                if upper.startswith("FIELDS "):
                    fields = line.split()[1:]
                    continue
                if upper.startswith("DATA "):
                    data_mode = line.split()[1].lower()
                    if data_mode != "ascii":
                        raise ValueError("only ASCII PCD files are supported: %s" % path)
                    break

            if not fields:
                raise ValueError("PCD file has no FIELDS line: %s" % path)
            try:
                x_idx = fields.index("x")
                y_idx = fields.index("y")
                z_idx = fields.index("z")
            except ValueError as exc:
                raise ValueError("PCD file must contain x y z fields: %s" % path) from exc
            if data_mode != "ascii":
                raise ValueError("PCD file has no DATA ascii section: %s" % path)

            for raw_line in f:
                line = raw_line.strip()
                if not line:
                    continue
                values = line.split()
                points.append(
                    (float(values[x_idx]), float(values[y_idx]), float(values[z_idx]))
                )

        return points


def main():
    rospy.init_node("pcd_pointcloud_publisher")
    PcdPointCloudPublisher().spin()


if __name__ == "__main__":
    main()
