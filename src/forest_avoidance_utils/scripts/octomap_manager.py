#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch octomap_server and save the built map when shutting down."""

import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from glob import glob

import roslib.packages
import rospy


def find_ros_executable(package, executable):
    """Resolve a package executable's absolute path without relying on rosrun.

    `rosrun` (ros-noetic-rosbash) is not installed on every machine, so spawning
    it via subprocess can fail with FileNotFoundError. roslib.packages.find_node
    looks up the binary the same way rosrun does internally.
    """
    matches = roslib.packages.find_node(package, executable)
    if not matches:
        raise RuntimeError(
            "could not find executable '%s' in package '%s'" % (executable, package))
    return matches[0]


class OctomapManager(object):
    def __init__(self):
        self.output_dir = os.path.expanduser(
            rospy.get_param("~output_dir", "~/octomap_maps"))
        self.filename_prefix = rospy.get_param("~filename_prefix", "forest_octomap")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/uav0/velodyne_points")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.resolution = float(rospy.get_param("~resolution", 0.15))
        self.sensor_model_min_range = float(rospy.get_param("~sensor_model_min_range", 0.0))
        self.max_range = float(rospy.get_param("~max_range", 8.0))
        self.pointcloud_min_z = float(rospy.get_param("~pointcloud_min_z", 0.05))
        self.pointcloud_max_z = float(rospy.get_param("~pointcloud_max_z", 6.0))
        self.occupancy_min_z = float(rospy.get_param("~occupancy_min_z", self.pointcloud_min_z))
        self.occupancy_max_z = float(rospy.get_param("~occupancy_max_z", self.pointcloud_max_z))
        self.filter_speckles = bool(rospy.get_param("~filter_speckles", False))
        self.filter_ground = bool(rospy.get_param("~filter_ground", False))
        self.ground_filter_distance = float(rospy.get_param("~ground_filter_distance", 0.04))
        self.ground_filter_angle = float(rospy.get_param("~ground_filter_angle", 0.15))
        self.ground_filter_plane_distance = float(rospy.get_param("~ground_filter_plane_distance", 0.07))
        self.sensor_model_hit = float(rospy.get_param("~sensor_model_hit", 0.80))
        self.sensor_model_miss = float(rospy.get_param("~sensor_model_miss", 0.48))
        self.sensor_model_min = float(rospy.get_param("~sensor_model_min", 0.12))
        self.sensor_model_max = float(rospy.get_param("~sensor_model_max", 0.97))
        self.save_timeout = float(rospy.get_param("~save_timeout", 12.0))
        self.startup_wait = float(rospy.get_param("~startup_wait", 1.0))
        self.autosave_interval = float(rospy.get_param("~autosave_interval", 15.0))
        self.cleanup_old_autosaves = bool(rospy.get_param("~cleanup_old_autosaves", True))
        self.save_on_shutdown = bool(rospy.get_param("~save_on_shutdown", False))
        self.autosave_updates_latest = bool(rospy.get_param("~autosave_updates_latest", False))
        self.last_autosave_time = time.time()
        self.proc = None
        self.saved = False
        self.saving = False

        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)
        if self.cleanup_old_autosaves:
            self.cleanup_legacy_autosaves()

        self.configure_octomap_server_params()
        self.start_octomap_server()
        rospy.on_shutdown(self.shutdown)
        signal.signal(signal.SIGINT, self.handle_signal)
        signal.signal(signal.SIGTERM, self.handle_signal)

    def configure_octomap_server_params(self):
        rospy.set_param("/octomap_server/frame_id", self.frame_id)
        rospy.set_param("/octomap_server/resolution", self.resolution)
        rospy.set_param("/octomap_server/sensor_model/min_range", self.sensor_model_min_range)
        rospy.set_param("/octomap_server/sensor_model/max_range", self.max_range)
        rospy.set_param("/octomap_server/pointcloud_min_z", self.pointcloud_min_z)
        rospy.set_param("/octomap_server/pointcloud_max_z", self.pointcloud_max_z)
        rospy.set_param("/octomap_server/occupancy_min_z", self.occupancy_min_z)
        rospy.set_param("/octomap_server/occupancy_max_z", self.occupancy_max_z)
        rospy.set_param("/octomap_server/filter_speckles", self.filter_speckles)
        rospy.set_param("/octomap_server/filter_ground", self.filter_ground)
        rospy.set_param("/octomap_server/ground_filter_distance", self.ground_filter_distance)
        rospy.set_param("/octomap_server/ground_filter_angle", self.ground_filter_angle)
        rospy.set_param("/octomap_server/ground_filter_plane_distance", self.ground_filter_plane_distance)
        rospy.set_param("/octomap_server/sensor_model/hit", self.sensor_model_hit)
        rospy.set_param("/octomap_server/sensor_model/miss", self.sensor_model_miss)
        rospy.set_param("/octomap_server/sensor_model/min", self.sensor_model_min)
        rospy.set_param("/octomap_server/sensor_model/max", self.sensor_model_max)
        rospy.set_param("/octomap_server/incremental_2D_projection", True)
        rospy.set_param("/octomap_server/compress_map", True)
        rospy.set_param("/octomap_server/latch", False)
        rospy.set_param("/octomap_server/base_frame_id", "base_link")

    def start_octomap_server(self):
        cmd = [
            find_ros_executable("octomap_server", "octomap_server_node"),
            "__name:=octomap_server",
            "cloud_in:=%s" % self.cloud_topic,
        ]
        rospy.logwarn("[OctomapManager] starting: %s", " ".join(cmd))
        self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        time.sleep(self.startup_wait)
        rospy.logwarn(
            "[OctomapManager] LiDAR mapping cloud=%s frame=%s resolution=%.2f range=%.2f..%.2f "
            "insert_z=%.2f..%.2f occ_z=%.2f..%.2f output_dir=%s",
            self.cloud_topic, self.frame_id, self.resolution, self.sensor_model_min_range,
            self.max_range, self.pointcloud_min_z, self.pointcloud_max_z,
            self.occupancy_min_z, self.occupancy_max_z, self.output_dir)

    def cleanup_legacy_autosaves(self):
        pattern = os.path.join(self.output_dir, "%s_autosave_*.bt" % self.filename_prefix)
        for path in glob(pattern):
            try:
                os.unlink(path)
            except OSError:
                pass

    def save_map(self, final=True):
        if (final and self.saved) or self.saving:
            return None
        self.saving = True

        if final:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            map_path = os.path.join(self.output_dir, "%s_%s.bt" % (self.filename_prefix, stamp))
        else:
            map_path = os.path.join(self.output_dir, "autosave_latest.bt")
        cmd = [find_ros_executable("octomap_server", "octomap_saver"), map_path]
        rospy.logwarn("[OctomapManager] saving map: %s", map_path)
        try:
            subprocess.check_call(cmd, timeout=self.save_timeout)
        except subprocess.TimeoutExpired:
            rospy.logerr("[OctomapManager] save timeout after %.1fs: %s", self.save_timeout, map_path)
            self.saving = False
            return None
        except subprocess.CalledProcessError as exc:
            rospy.logerr("[OctomapManager] save failed rc=%s: %s", exc.returncode, map_path)
            self.saving = False
            return None

        if final or self.autosave_updates_latest:
            latest_path = os.path.join(self.output_dir, "latest.bt")
            try:
                if os.path.islink(latest_path) or os.path.exists(latest_path):
                    os.unlink(latest_path)
                os.symlink(os.path.basename(map_path), latest_path)
            except OSError:
                pass

        if final:
            self.saved = True
        self.saving = False
        rospy.logwarn("[OctomapManager] saved map -> %s", map_path)
        return map_path

    def stop_octomap_server(self):
        if self.proc is None or self.proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            self.proc.wait(timeout=3.0)
        except Exception:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            except Exception:
                pass

    def shutdown(self):
        if self.save_on_shutdown:
            self.save_map()
        self.stop_octomap_server()

    def handle_signal(self, signum, _frame):
        rospy.logwarn("[OctomapManager] received signal %s, shutting down", signum)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        self.shutdown()
        rospy.signal_shutdown("signal %s" % signum)
        sys.exit(0)

    def spin(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.proc is not None and self.proc.poll() is not None:
                rospy.logerr("[OctomapManager] octomap_server exited with code %s", self.proc.returncode)
                return
            if self.autosave_interval > 0.0 and time.time() - self.last_autosave_time >= self.autosave_interval:
                self.save_map(final=False)
                self.last_autosave_time = time.time()
            rate.sleep()


def main():
    rospy.init_node("octomap_manager")
    manager = OctomapManager()
    manager.spin()


if __name__ == "__main__":
    main()
