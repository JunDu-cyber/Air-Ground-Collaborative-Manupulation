#!/usr/bin/env python3
"""Save the OctoMap 2D projected_map as a standard ROS map (yaml + pgm).

Subscribes to /octomap_server/projected_map (nav_msgs/OccupancyGrid)
and saves it periodically and on shutdown as a ground-robot-compatible map.

Usage:
  rosrun forest_avoidance_utils map2d_saver.py \
      _output_dir:=$HOME/ego_ws/octomap_maps \
      _save_interval:=30.0 \
      _filename:=forest_2d_map
"""

import os
import sys
import time
import signal
import struct
import zlib
from datetime import datetime

import rospy
from nav_msgs.msg import OccupancyGrid


class Map2DSaver(object):
    def __init__(self):
        self.output_dir = os.path.expanduser(
            rospy.get_param("~output_dir", os.path.expanduser("~/octomap_maps")))
        self.filename = rospy.get_param("~filename", "forest_2d_map")
        self.save_interval = float(rospy.get_param("~save_interval", 30.0))
        self.map_topic = rospy.get_param("~map_topic", "/octomap_server/projected_map")
        self.free_threshold = float(rospy.get_param("~free_threshold", 0.196))
        self.occupied_threshold = float(rospy.get_param("~occupied_threshold", 0.65))
        self.save_on_shutdown = rospy.get_param("~save_on_shutdown", True)

        self.latest_map = None
        self.last_save_time = time.time()
        self.save_count = 0

        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_cb, queue_size=1)

        if self.save_on_shutdown:
            rospy.on_shutdown(self.shutdown_save)
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

        rospy.logwarn("[Map2DSaver] topic=%s output_dir=%s interval=%.0fs filename=%s",
                      self.map_topic, self.output_dir, self.save_interval, self.filename)

    def map_cb(self, msg):
        self.latest_map = msg

    def _handle_signal(self, signum, _frame):
        rospy.logwarn("[Map2DSaver] signal %s, saving final map", signum)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        self.save_map(tag="final")
        rospy.signal_shutdown("signal %s" % signum)
        sys.exit(0)

    def shutdown_save(self):
        self.save_map(tag="final")

    def save_map(self, tag=None):
        """Save the latest OccupancyGrid as PGM + YAML."""
        if self.latest_map is None:
            rospy.logwarn("[Map2DSaver] no map received yet, skipping save")
            return None

        msg = self.latest_map
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        if width == 0 or height == 0:
            rospy.logwarn("[Map2DSaver] empty map (%dx%d), skipping", width, height)
            return None

        # Generate filenames
        if tag:
            base = "%s_%s" % (self.filename, tag)
        else:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base = "%s_%s" % (self.filename, stamp)

        pgm_name = base + ".pgm"
        yaml_name = base + ".yaml"
        pgm_path = os.path.join(self.output_dir, pgm_name)
        yaml_path = os.path.join(self.output_dir, yaml_name)

        # Convert OccupancyGrid to PGM (same convention as map_saver)
        # ROS map: 0=free(white,254), 100=occupied(black,0), -1=unknown(gray,205)
        pixels = bytearray(width * height)
        for i, cell in enumerate(msg.data):
            if cell < 0:  # unknown
                pixels[i] = 205
            elif cell <= self.free_threshold * 100:
                pixels[i] = 254
            elif cell >= self.occupied_threshold * 100:
                pixels[i] = 0
            else:
                # Scale linearly between free and occupied
                pixels[i] = max(0, min(254, int(254.0 * (1.0 - cell / 100.0))))

        # Write PGM (binary, P5 format) - rows are flipped (bottom-to-top for ROS)
        with open(pgm_path, 'wb') as f:
            f.write(("P5\n%d %d\n255\n" % (width, height)).encode('ascii'))
            for row in range(height - 1, -1, -1):
                f.write(bytes(pixels[row * width:(row + 1) * width]))

        # Write PNG (color-coded: black=occupied, white=free, gray=unknown)
        png_path = os.path.join(self.output_dir, base + ".png")
        self._write_png(png_path, pixels, width, height)

        # Write YAML
        ox = origin.position.x
        oy = origin.position.y
        # Extract yaw from quaternion
        q = origin.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        import math
        yaw = math.atan2(siny, cosy)

        with open(yaml_path, 'w') as f:
            f.write("image: %s\n" % pgm_name)
            f.write("resolution: %.6f\n" % resolution)
            f.write("origin: [%.6f, %.6f, %.6f]\n" % (ox, oy, yaw))
            f.write("negate: 0\n")
            f.write("occupied_thresh: %.2f\n" % self.occupied_threshold)
            f.write("free_thresh: %.2f\n" % self.free_threshold)

        # Update latest symlinks
        for ext in (".yaml", ".pgm", ".png"):
            latest = os.path.join(self.output_dir, self.filename + "_latest" + ext)
            try:
                if os.path.islink(latest) or os.path.exists(latest):
                    os.unlink(latest)
                os.symlink(base + ext, latest)
            except OSError:
                pass

        self.save_count += 1
        rospy.logwarn("[Map2DSaver] saved 2D map #%d: %s (%dx%d, res=%.3f) + PNG",
                      self.save_count, yaml_path, width, height, resolution)
        return yaml_path

    def _write_png(self, path, pixels, width, height):
        """Write a color PNG: occupied=black, free=white, unknown=light gray."""
        def _make_chunk(chunk_type, data):
            raw = chunk_type + data
            return struct.pack('>I', len(data)) + raw + struct.pack('>I', zlib.crc32(raw) & 0xFFFFFFFF)

        # Build RGB rows (bottom-to-top, same as PGM)
        raw_data = bytearray()
        for row in range(height - 1, -1, -1):
            raw_data.append(0)  # PNG filter: None
            for col in range(width):
                val = pixels[row * width + col]
                if val == 0:       # occupied -> black
                    raw_data.extend(b'\x00\x00\x00')
                elif val == 254:   # free -> white
                    raw_data.extend(b'\xff\xff\xff')
                elif val == 205:   # unknown -> light gray
                    raw_data.extend(b'\xc8\xc8\xc8')
                else:              # intermediate -> proportional gray
                    raw_data.extend(bytes([val, val, val]))

        with open(path, 'wb') as f:
            # PNG signature
            f.write(b'\x89PNG\r\n\x1a\n')
            # IHDR
            ihdr = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
            f.write(_make_chunk(b'IHDR', ihdr))
            # IDAT
            compressed = zlib.compress(bytes(raw_data), 9)
            f.write(_make_chunk(b'IDAT', compressed))
            # IEND
            f.write(_make_chunk(b'IEND', b''))

    def spin(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if (self.save_interval > 0.0
                    and time.time() - self.last_save_time >= self.save_interval
                    and self.latest_map is not None):
                self.save_map()
                self.last_save_time = time.time()
            rate.sleep()


def main():
    rospy.init_node("map2d_saver")
    saver = Map2DSaver()
    saver.spin()


if __name__ == "__main__":
    main()
