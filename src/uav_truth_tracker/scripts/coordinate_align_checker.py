#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""coordinate_align_checker.py

订阅 UAV GPS 与 UGV GPS，各自转 UTM，计算水平距离偏差，判断两套坐标系是否对齐
（方案①：以 GPS/UTM 为公共参考）。
- 偏差 < align_threshold(0.5m)  -> loginfo  ALIGNED
- 偏差 > warn_threshold(2.0m)   -> logwarn  MISALIGNED
- 之间                          -> loginfo  MARGINAL
发布 /coordinate_align/status (std_msgs/String, JSON)。

本节点比较"两台车 GPS 的 UTM 距离"。它能直接抓到本系统最典型的 bug：
UAV 用 PX4 默认 home(苏黎世 47.40,8.55) 而 UGV 用 hector 参考(49.9,8.9) →
两者相差 ~280km → MISALIGNED。修好后（两侧同 datum / 同 home）偏差即为两机真实间距。

UTM 换算自实现（标准横轴墨卡托正算），离线、无第三方依赖。
"""

import json
import math

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


def ll_to_utm(lat_deg, lon_deg):
    """WGS84 经纬度 -> UTM(easting, northing, zone)。离线、无依赖。"""
    a = 6378137.0                      # WGS84 长半轴
    f = 1.0 / 298.257223563            # 扁率
    k0 = 0.9996                        # UTM 中央经线尺度
    e2 = f * (2 - f)                   # 第一偏心率平方
    ep2 = e2 / (1 - e2)                # 第二偏心率平方
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    zone = int((lon_deg + 180) / 6) + 1
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)   # 该带中央经线
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
        northing += 10000000.0         # 南半球偏移
    return easting, northing, zone


class CoordinateAlignChecker(object):
    def __init__(self):
        self.uav_topic = rospy.get_param("~uav_gps_topic", "/mavros/global_position/global")
        self.ugv_topic = rospy.get_param("~ugv_gps_topic", "navsat/fix")
        self.align_thr = float(rospy.get_param("~align_threshold", 0.5))   # m
        self.warn_thr = float(rospy.get_param("~warn_threshold", 2.0))     # m
        self.baseline = float(rospy.get_param("~baseline_offset", 0.0))    # 已知两天线基线(m)
        self.rate_hz = float(rospy.get_param("~rate", 1.0))
        self.fix_timeout = float(rospy.get_param("~fix_timeout", 5.0))     # 数据过期阈值(s)

        self.uav_fix = None
        self.ugv_fix = None
        self.uav_stamp = None
        self.ugv_stamp = None

        self.pub = rospy.Publisher("/coordinate_align/status", String, queue_size=1)
        rospy.Subscriber(self.uav_topic, NavSatFix, self._uav_cb, queue_size=1)
        rospy.Subscriber(self.ugv_topic, NavSatFix, self._ugv_cb, queue_size=1)
        rospy.loginfo("[align] UAV=%s  UGV=%s  align<%.2fm warn>%.2fm baseline=%.2fm",
                      self.uav_topic, self.ugv_topic, self.align_thr, self.warn_thr, self.baseline)

    def _valid(self, msg):
        return (msg.status.status >= 0
                and not math.isnan(msg.latitude) and not math.isnan(msg.longitude))

    def _uav_cb(self, msg):
        if self._valid(msg):
            self.uav_fix, self.uav_stamp = msg, rospy.Time.now()

    def _ugv_cb(self, msg):
        if self._valid(msg):
            self.ugv_fix, self.ugv_stamp = msg, rospy.Time.now()

    def _fresh(self, stamp):
        return stamp is not None and (rospy.Time.now() - stamp).to_sec() < self.fix_timeout

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self._tick()
            rate.sleep()

    def _tick(self):
        status = {"state": "WAITING", "deviation_m": None,
                  "uav_utm": None, "ugv_utm": None, "zone_match": None,
                  "stamp": rospy.Time.now().to_sec()}

        if not (self._fresh(self.uav_stamp) and self._fresh(self.ugv_stamp)):
            status["state"] = "NO_FIX"
            rospy.logwarn_throttle(5.0, "[align] 等待有效 GPS（UAV/UGV 至少一方无新数据）")
            self.pub.publish(String(data=json.dumps(status)))
            return

        eu, nu, zu = ll_to_utm(self.uav_fix.latitude, self.uav_fix.longitude)
        eg, ng, zg = ll_to_utm(self.ugv_fix.latitude, self.ugv_fix.longitude)
        raw = math.hypot(eu - eg, nu - ng)
        dev = abs(raw - self.baseline)        # 扣掉已知基线后的"对齐偏差"

        status["uav_utm"] = [round(eu, 3), round(nu, 3), zu]
        status["ugv_utm"] = [round(eg, 3), round(ng, 3), zg]
        status["zone_match"] = (zu == zg)
        status["deviation_m"] = round(dev, 3)

        if zu != zg:
            status["state"] = "ZONE_MISMATCH"
            rospy.logwarn_throttle(5.0, "[align] UTM zone 不一致 UAV=%d UGV=%d，需选同带 datum", zu, zg)
        elif dev < self.align_thr:
            status["state"] = "ALIGNED"
            rospy.loginfo_throttle(5.0, "[align] ALIGNED 偏差=%.3fm (<%.2f) OK", dev, self.align_thr)
        elif dev > self.warn_thr:
            status["state"] = "MISALIGNED"
            rospy.logwarn_throttle(2.0, "[align] MISALIGNED 偏差=%.3fm (>%.2f) 检查两侧 datum/home 是否一致",
                                   dev, self.warn_thr)
        else:
            status["state"] = "MARGINAL"
            rospy.loginfo_throttle(5.0, "[align] MARGINAL 偏差=%.3fm", dev)

        self.pub.publish(String(data=json.dumps(status)))


def main():
    rospy.init_node("coordinate_align_checker")
    CoordinateAlignChecker().spin()


if __name__ == "__main__":
    main()
