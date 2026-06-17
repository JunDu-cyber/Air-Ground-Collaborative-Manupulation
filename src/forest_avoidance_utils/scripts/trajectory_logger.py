#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Log EGO desired trajectory and PX4 actual pose for tracking analysis."""

import csv
import math
import os
import threading
from datetime import datetime

import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from tf.transformations import euler_from_quaternion


class TrajectoryLogger(object):
    def __init__(self):
        self.actual_topic = rospy.get_param("~actual_topic", "/mavros/local_position/pose")
        self.desired_topic = rospy.get_param("~desired_topic", "/planning/pos_cmd")
        self.output_dir = os.path.expanduser(
            rospy.get_param("~output_dir", "~/trajectory_logs"))
        self.max_cmd_age = float(rospy.get_param("~max_cmd_age", 0.5))
        self.flush_every = int(rospy.get_param("~flush_every", 25))

        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.output_dir, "flight_trajectory_%s.csv" % stamp)
        self.lock = threading.Lock()
        self.last_cmd = None
        self.last_cmd_time = None
        self.rows_written = 0
        self.t0 = None

        self.csv_file = open(self.csv_path, "w")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "time_ros", "time_rel",
            "actual_x", "actual_y", "actual_z",
            "actual_roll", "actual_pitch", "actual_yaw",
            "desired_x", "desired_y", "desired_z",
            "desired_vx", "desired_vy", "desired_vz",
            "desired_yaw", "trajectory_id", "trajectory_flag", "cmd_age",
            "error_x", "error_y", "error_z", "error_xy", "error_3d",
        ])
        self.csv_file.flush()

        rospy.Subscriber(self.desired_topic, PositionCommand, self.cmd_cb, queue_size=50)
        rospy.Subscriber(self.actual_topic, PoseStamped, self.pose_cb, queue_size=200)

        rospy.on_shutdown(self.close)
        rospy.logwarn("[TrajectoryLogger] logging %s + %s -> %s",
                      self.actual_topic, self.desired_topic, self.csv_path)

    def cmd_cb(self, msg):
        with self.lock:
            self.last_cmd = msg
            self.last_cmd_time = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()

    def pose_cb(self, msg):
        t = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        t_sec = t.to_sec()
        if self.t0 is None:
            self.t0 = t_sec

        p = msg.pose.position
        q = msg.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        with self.lock:
            cmd = self.last_cmd
            cmd_time = self.last_cmd_time

        row = [
            "%.6f" % t_sec, "%.6f" % (t_sec - self.t0),
            "%.6f" % p.x, "%.6f" % p.y, "%.6f" % p.z,
            "%.6f" % roll, "%.6f" % pitch, "%.6f" % yaw,
        ]

        if cmd is None or cmd_time is None:
            row += [""] * 15
        else:
            cmd_age = max(0.0, (t - cmd_time).to_sec())
            if cmd_age <= self.max_cmd_age:
                dp = cmd.position
                dv = cmd.velocity
                ex = p.x - dp.x
                ey = p.y - dp.y
                ez = p.z - dp.z
                e_xy = math.hypot(ex, ey)
                e_3d = math.sqrt(ex * ex + ey * ey + ez * ez)
                row += [
                    "%.6f" % dp.x, "%.6f" % dp.y, "%.6f" % dp.z,
                    "%.6f" % dv.x, "%.6f" % dv.y, "%.6f" % dv.z,
                    "%.6f" % cmd.yaw, str(cmd.trajectory_id), str(cmd.trajectory_flag),
                    "%.6f" % cmd_age,
                    "%.6f" % ex, "%.6f" % ey, "%.6f" % ez,
                    "%.6f" % e_xy, "%.6f" % e_3d,
                ]
            else:
                row += [""] * 15

        self.writer.writerow(row)
        self.rows_written += 1
        if self.rows_written % self.flush_every == 0:
            self.csv_file.flush()

    def close(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
            rospy.logwarn("[TrajectoryLogger] saved %d rows -> %s", self.rows_written, self.csv_path)
        except Exception:
            pass


def main():
    rospy.init_node("trajectory_logger")
    TrajectoryLogger()
    rospy.spin()


if __name__ == "__main__":
    main()
