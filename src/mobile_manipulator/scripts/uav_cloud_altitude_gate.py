#!/usr/bin/env python3
"""Gate the UAV LiDAR cloud into elevation_mapping by UAV altitude.

Before takeoff the UAV's 16-line Velodyne sits ~on the ground and produces a
degenerate scan: concentric ground rings + a max-range rim that elevation_mapping
fuses into a floating bowl/ring "above" the UGV. This relay forwards the cloud
ONLY while the UAV is airborne above `min_altitude` (MAVROS local z, i.e. height
above the takeoff ground), and clears the elevation map ONCE when it first lifts
off so any pre-takeoff residue is wiped. While the UAV is low (on ground, taking
off, landing) the cloud is dropped, so the map only ever sees the clean
look-straight-down scan.
"""
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


class AltitudeGate:
    def __init__(self):
        self.min_alt = rospy.get_param('~min_altitude', 5.0)        # [m] MAVROS local z
        self.odom_topic = rospy.get_param('~odom_topic', '/mavros/local_position/odom')
        self.clear_on_open = rospy.get_param('~clear_on_open', True)
        self.clear_service = rospy.get_param('~clear_service', '/elevation_mapping/clear_map')
        self.z = None
        self.opened = False
        self.pub = rospy.Publisher('~output', PointCloud2, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=10)
        rospy.Subscriber('~input', PointCloud2, self._cloud_cb, queue_size=2)
        rospy.loginfo('[altitude_gate] gating cloud into elevation_mapping at z >= %.1f m (odom %s)',
                      self.min_alt, self.odom_topic)

    def _odom_cb(self, msg):
        self.z = msg.pose.pose.position.z

    def _cloud_cb(self, msg):
        if self.z is None or self.z < self.min_alt:
            return  # UAV not airborne -> drop the degenerate on-ground scan
        if not self.opened:
            self.opened = True
            rospy.loginfo('[altitude_gate] UAV airborne (z=%.2f m) -> fusing cloud', self.z)
            if self.clear_on_open:
                try:
                    rospy.wait_for_service(self.clear_service, timeout=2.0)
                    rospy.ServiceProxy(self.clear_service, Empty)()
                    rospy.loginfo('[altitude_gate] cleared elevation map (wiped pre-takeoff residue)')
                except Exception as exc:
                    rospy.logwarn('[altitude_gate] clear_map failed: %s', exc)
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('uav_cloud_altitude_gate')
    AltitudeGate()
    rospy.spin()
