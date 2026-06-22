#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_map_origin_saver.py

建图时自动记录【UAV 的 map 原点(=PX4本地原点=起飞点) 的 UTM 坐标】到一个 sidecar
文件，供 UGV 侧 pcd_to_occupancy_map.py 开机自动算 GPS 锚定偏移——用户不用手填任何
经纬度/offset，"一启动就配好"。

原理：MAVROS 同一时刻给出
  /mavros/global_position/global  -> 当前 GPS(lat,lon)
  /mavros/local_position/odom     -> 当前相对起飞点的 ENU 位移 (x=东, y=北)
UTM 也是东/北，所以 UAV map 原点的 UTM =
  origin_E = UTM(gps).E - local_x ;  origin_N = UTM(gps).N - local_y
把它写进 ~/pointcloud_maps/uav_map_origin.yaml。
"""

import math
import os

import rospy
import yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


def ll_to_utm(lat_deg, lon_deg):
    a = 6378137.0
    f = 1.0 / 298.257223563
    k0 = 0.9996
    e2 = f * (2 - f)
    ep2 = e2 / (1 - e2)
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    zone = int((lon_deg + 180) / 6) + 1
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)
    N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    T = math.tan(lat) ** 2
    C = ep2 * math.cos(lat) ** 2
    A = math.cos(lat) * (lon - lon0)
    M = a * ((1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * lat
             - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * lat)
             + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * lat)
             - (35 * e2**3 / 3072) * math.sin(6 * lat))
    easting = (k0 * N * (A + (1 - T + C) * A**3 / 6
               + (5 - 18 * T + T**2 + 72 * C - 58 * ep2) * A**5 / 120) + 500000.0)
    northing = (k0 * (M + N * math.tan(lat) * (A**2 / 2
                + (5 - T + 9 * C + 4 * C**2) * A**4 / 24
                + (61 - 58 * T + T**2 + 600 * C - 330 * ep2) * A**6 / 720)))
    if lat_deg < 0:
        northing += 10000000.0
    return easting, northing, zone


def compute_origin_utm(lat, lon, local_x, local_y):
    """由 (当前GPS, 当前本地ENU位移) 反推 UAV map 原点的 UTM。"""
    e, n, zone = ll_to_utm(lat, lon)
    return e - local_x, n - local_y, zone


class UavMapOriginSaver(object):
    def __init__(self):
        self.out = os.path.expanduser(rospy.get_param(
            "~origin_file", "~/pointcloud_maps/uav_map_origin.yaml"))
        self.fix = None
        self.odom = None
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self._fix_cb, queue_size=5)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._odom_cb, queue_size=5)
        rospy.Timer(rospy.Duration(2.0), self._save)
        rospy.loginfo("[uav_map_origin_saver] -> %s", self.out)

    def _fix_cb(self, m):
        if m.status.status >= 0 and not math.isnan(m.latitude) and not math.isnan(m.longitude):
            self.fix = m

    def _odom_cb(self, m):
        self.odom = m

    def _save(self, _evt):
        if self.fix is None or self.odom is None:
            rospy.logwarn_throttle(10.0, "[uav_map_origin_saver] 等待 GPS+local odom ...")
            return
        lx = self.odom.pose.pose.position.x
        ly = self.odom.pose.pose.position.y
        e, n, zone = compute_origin_utm(self.fix.latitude, self.fix.longitude, lx, ly)
        data = {"utm_easting": float(e), "utm_northing": float(n), "utm_zone": int(zone),
                "sample_lat": float(self.fix.latitude), "sample_lon": float(self.fix.longitude),
                "local_x": float(lx), "local_y": float(ly),
                "stamp": rospy.Time.now().to_sec()}
        d = os.path.dirname(self.out)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        tmp = self.out + ".tmp"
        with open(tmp, "w") as f:
            yaml.safe_dump(data, f)
        os.replace(tmp, self.out)
        rospy.loginfo_throttle(30.0, "[uav_map_origin_saver] origin UTM=(%.1f,%.1f) zone%d saved",
                               e, n, zone)


def main():
    rospy.init_node("uav_map_origin_saver")
    UavMapOriginSaver()
    rospy.spin()


if __name__ == "__main__":
    main()
