#!/usr/bin/env python3
"""Convert an OctoMap .bt file to a 2D PNG/PGM/YAML map.

Usage:
  rosrun forest_avoidance_utils bt_to_2dmap.py /path/to/map.bt [--min_z 0.05] [--max_z 1.20]
"""

import os
import sys
import time
import signal
import struct
import zlib
import subprocess
import argparse
import math

import roslib.packages
import rospy
from nav_msgs.msg import OccupancyGrid


def find_ros_executable(package, executable):
    """Resolve a package executable's absolute path (rosrun may not be installed)."""
    matches = roslib.packages.find_node(package, executable)
    if not matches:
        raise RuntimeError(
            "could not find executable '%s' in package '%s'" % (executable, package))
    return matches[0]


class BtTo2DMap(object):
    def __init__(self, bt_path, output_dir, min_z, max_z):
        self.bt_path = os.path.abspath(bt_path)
        self.output_dir = output_dir or os.path.dirname(self.bt_path)
        self.min_z = min_z
        self.max_z = max_z
        self.received_map = None
        self.proc = None

    def run(self):
        rospy.init_node("bt_to_2dmap", anonymous=True)

        # Subscribe BEFORE starting octomap_server (it latches)
        rospy.Subscriber("/projected_map", OccupancyGrid, self._map_cb, queue_size=1)

        # Start octomap_server with the .bt file
        cmd = [
            find_ros_executable("octomap_server", "octomap_server_node"),
            self.bt_path,
            "__name:=octomap_server_convert",
            "_frame_id:=map",
            "_occupancy_min_z:=%.4f" % self.min_z,
            "_occupancy_max_z:=%.4f" % self.max_z,
            "_latch:=true",
        ]
        rospy.logwarn("[BtTo2DMap] loading %s (z: %.2f ~ %.2f)", self.bt_path, self.min_z, self.max_z)
        self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)

        # Wait for the projected map
        timeout = time.time() + 20.0
        rate = rospy.Rate(5)
        while self.received_map is None and time.time() < timeout and not rospy.is_shutdown():
            rate.sleep()

        if self.received_map is None:
            rospy.logerr("[BtTo2DMap] timeout waiting for /projected_map!")
            self._kill_server()
            return False

        result = self._save_map(self.received_map)
        self._kill_server()
        return result

    def _map_cb(self, msg):
        if msg.info.width > 0 and msg.info.height > 0:
            self.received_map = msg
            rospy.logwarn("[BtTo2DMap] received map %dx%d", msg.info.width, msg.info.height)

    def _kill_server(self):
        if self.proc and self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                self.proc.wait(timeout=3.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                except Exception:
                    pass

    def _save_map(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        base = os.path.splitext(os.path.basename(self.bt_path))[0] + "_2d"
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

        # Convert to pixels
        pixels = bytearray(width * height)
        for i, cell in enumerate(msg.data):
            if cell < 0:
                pixels[i] = 205
            elif cell <= 19:
                pixels[i] = 254
            elif cell >= 65:
                pixels[i] = 0
            else:
                pixels[i] = max(0, min(254, int(254.0 * (1.0 - cell / 100.0))))

        # PGM
        pgm_path = os.path.join(self.output_dir, base + ".pgm")
        with open(pgm_path, 'wb') as f:
            f.write(("P5\n%d %d\n255\n" % (width, height)).encode('ascii'))
            for row in range(height - 1, -1, -1):
                f.write(bytes(pixels[row * width:(row + 1) * width]))

        # PNG
        png_path = os.path.join(self.output_dir, base + ".png")
        self._write_png(png_path, pixels, width, height)

        # YAML
        yaml_path = os.path.join(self.output_dir, base + ".yaml")
        q = origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with open(yaml_path, 'w') as f:
            f.write("image: %s\n" % (base + ".pgm"))
            f.write("resolution: %.6f\n" % resolution)
            f.write("origin: [%.6f, %.6f, %.6f]\n" % (origin.position.x, origin.position.y, yaw))
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.196\n")

        rospy.logwarn("[BtTo2DMap] saved PNG: %s", png_path)
        rospy.logwarn("[BtTo2DMap] saved YAML: %s", yaml_path)
        rospy.logwarn("[BtTo2DMap] saved PGM: %s", pgm_path)
        return True

    def _write_png(self, path, pixels, width, height):
        def _chunk(ctype, data):
            raw = ctype + data
            return struct.pack('>I', len(data)) + raw + struct.pack('>I', zlib.crc32(raw) & 0xFFFFFFFF)

        raw_data = bytearray()
        for row in range(height - 1, -1, -1):
            raw_data.append(0)
            for col in range(width):
                v = pixels[row * width + col]
                if v == 0:
                    raw_data.extend(b'\x00\x00\x00')
                elif v == 254:
                    raw_data.extend(b'\xff\xff\xff')
                elif v == 205:
                    raw_data.extend(b'\xc8\xc8\xc8')
                else:
                    raw_data.extend(bytes([v, v, v]))

        with open(path, 'wb') as f:
            f.write(b'\x89PNG\r\n\x1a\n')
            f.write(_chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)))
            f.write(_chunk(b'IDAT', zlib.compress(bytes(raw_data), 9)))
            f.write(_chunk(b'IEND', b''))


def main():
    parser = argparse.ArgumentParser(description="Convert .bt to 2D map PNG/PGM/YAML")
    parser.add_argument("bt_file", help="Path to .bt file")
    parser.add_argument("--output_dir", "-o", default=None)
    parser.add_argument("--min_z", type=float, default=0.05)
    parser.add_argument("--max_z", type=float, default=1.20)
    args_filtered = [a for a in sys.argv[1:] if not a.startswith("__")]
    args = parser.parse_args(args_filtered)

    if not os.path.isfile(args.bt_file):
        print("Error: file not found: %s" % args.bt_file)
        sys.exit(1)

    converter = BtTo2DMap(args.bt_file, args.output_dir, args.min_z, args.max_z)
    success = converter.run()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
