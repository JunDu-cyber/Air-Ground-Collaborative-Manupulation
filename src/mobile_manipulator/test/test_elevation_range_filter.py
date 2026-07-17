#!/usr/bin/env python3
import math
import threading
import unittest

import rospy
import rostest
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


FIELDS = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('intensity', 12, PointField.FLOAT32, 1),
]


class ElevationRangeFilterTest(unittest.TestCase):
    def setUp(self):
        self.pub = rospy.Publisher('/test/range/input', PointCloud2, queue_size=1)
        self.missing_pub = rospy.Publisher('/test/missing_tf/input', PointCloud2,
                                           queue_size=1)
        self.output_event = threading.Event()
        self.missing_event = threading.Event()
        self.output_msg = None
        self.missing_msg = None
        rospy.Subscriber('/test/range/output', PointCloud2, self._output_cb,
                         queue_size=1)
        rospy.Subscriber('/test/missing_tf/output', PointCloud2, self._missing_cb,
                         queue_size=1)

    def _output_cb(self, msg):
        self.output_msg = msg
        self.output_event.set()

    def _missing_cb(self, msg):
        self.missing_msg = msg
        self.missing_event.set()

    @staticmethod
    def cloud():
        header = Header(stamp=rospy.Time(0), frame_id='velodyne')
        return pc2.create_cloud(header, FIELDS, [
            (3.0, 4.0, 1.0, 11.0),       # 5 m: keep
            (20.0, 0.0, -2.0, 22.0),     # boundary: keep
            (20.01, 0.0, 0.0, 33.0),     # outside: drop
            (math.nan, 0.0, 0.0, 44.0),  # non-finite: drop
        ])

    def publish_until_received(self, pub, event, message_attr):
        deadline = rospy.Time.now() + rospy.Duration(8.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if pub.get_num_connections() == 0:
                rospy.sleep(0.1)
                continue
            pub.publish(ElevationRangeFilterTest.cloud())
            if event.wait(0.5):
                return getattr(self, message_attr)
        raise AssertionError('timed out waiting for filter output')

    def test_range_and_field_preservation(self):
        output = self.publish_until_received(self.pub, self.output_event, 'output_msg')
        self.assertEqual([field.name for field in output.fields],
                         ['x', 'y', 'z', 'intensity'])
        points = list(pc2.read_points(output,
                                      field_names=('x', 'y', 'z', 'intensity'),
                                      skip_nans=False))
        self.assertEqual(len(points), 2)
        self.assertEqual([point[3] for point in points], [11.0, 22.0])

    def test_missing_tf_withholds_scan(self):
        output = self.publish_until_received(self.missing_pub, self.missing_event,
                                             'missing_msg')
        self.assertEqual(output.width, 0)
        self.assertEqual(len(output.data), 0)


if __name__ == '__main__':
    rospy.init_node('test_elevation_range_filter')
    rostest.rosrun('mobile_manipulator', 'elevation_range_filter',
                   ElevationRangeFilterTest)
