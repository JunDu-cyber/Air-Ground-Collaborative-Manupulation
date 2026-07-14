#!/usr/bin/env python3
"""Wall-clock health diagnostics for the always-on UAV mine RGB-D camera."""

import collections
import copy
import json
import math
import threading
import time

import rospy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CameraInfo, Image


class StreamState:
    def __init__(self):
        self.message = None
        self.last_arrival_wall = None
        self.arrivals = collections.deque()
        self.last_stamp = None
        self.last_stamp_advance_wall = None
        self.stamp_repeats = 0
        self.regressions = 0

    def update(self, message, now_wall, window):
        stamp = message.header.stamp.to_sec()
        if self.last_stamp is not None:
            if stamp > self.last_stamp:
                self.last_stamp_advance_wall = now_wall
            elif stamp < self.last_stamp:
                self.regressions += 1
            else:
                self.stamp_repeats += 1
        self.last_stamp = stamp
        self.message = copy.deepcopy(message)
        self.last_arrival_wall = now_wall
        self.arrivals.append(now_wall)
        cutoff = now_wall - window
        while self.arrivals and self.arrivals[0] < cutoff:
            self.arrivals.popleft()

    def rate(self):
        if len(self.arrivals) < 2:
            return 0.0
        duration = self.arrivals[-1] - self.arrivals[0]
        return (len(self.arrivals) - 1) / duration if duration > 1e-6 else 0.0

    def stamp_is_advancing(self, now_wall, maximum_age):
        return bool(
            self.last_stamp_advance_wall is not None
            and now_wall - self.last_stamp_advance_wall <= maximum_age
        )

    def stamp_advance_age(self, now_wall):
        if self.last_stamp_advance_wall is None:
            return math.inf
        return max(0.0, now_wall - self.last_stamp_advance_wall)


class MineCameraDiagnostics:
    def __init__(self):
        self.rgb_topic = rospy.get_param(
            "~rgb_topic", "/mine_camera/rgb/image_raw"
        )
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/mine_camera/depth/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/mine_camera/rgb/camera_info"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/mine_camera/diagnostics"
        )
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.camera_frame = rospy.get_param(
            "~camera_frame", "mine_camera_optical_frame"
        )
        self.window = max(float(rospy.get_param("~rate_window", 5.0)), 2.0)
        self.maximum_age = max(
            float(rospy.get_param("~maximum_frame_age_wall", 2.0)), 0.5
        )
        self.maximum_stamp_delta = max(
            float(rospy.get_param("~maximum_rgb_depth_stamp_delta", 0.05)),
            0.001,
        )
        self.minimum_rate = max(
            float(rospy.get_param("~minimum_wall_rate", 0.5)), 0.01
        )
        self.tf_timeout = max(float(rospy.get_param("~tf_timeout", 0.05)), 0.0)
        self.lock = threading.RLock()
        self.rgb = StreamState()
        self.depth = StreamState()
        self.camera_info = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(60.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            self.output_topic, DiagnosticArray, queue_size=2, latch=True
        )
        rospy.Subscriber(self.rgb_topic, Image, self._rgb_cb, queue_size=10)
        rospy.Subscriber(self.depth_topic, Image, self._depth_cb, queue_size=10)
        rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._info_cb, queue_size=2
        )
        self.worker = threading.Thread(target=self._wall_loop, daemon=True)
        self.worker.start()

    def _rgb_cb(self, msg):
        with self.lock:
            self.rgb.update(msg, time.monotonic(), self.window)

    def _depth_cb(self, msg):
        with self.lock:
            self.depth.update(msg, time.monotonic(), self.window)

    def _info_cb(self, msg):
        with self.lock:
            self.camera_info = copy.deepcopy(msg)

    @staticmethod
    def _value(key, value):
        if isinstance(value, (list, tuple, dict)):
            value = json.dumps(value, sort_keys=True)
        return KeyValue(key=str(key), value=str(value))

    def _snapshot(self):
        now = time.monotonic()
        with self.lock:
            rgb = copy.deepcopy(self.rgb.message)
            depth = copy.deepcopy(self.depth.message)
            info = copy.deepcopy(self.camera_info)
            rgb_age = (
                math.inf if self.rgb.last_arrival_wall is None
                else now - self.rgb.last_arrival_wall
            )
            depth_age = (
                math.inf if self.depth.last_arrival_wall is None
                else now - self.depth.last_arrival_wall
            )
            values = {
                "rgb_wall_rate_hz": self.rgb.rate(),
                "depth_wall_rate_hz": self.depth.rate(),
                "rgb_last_frame_age_wall_s": rgb_age,
                "depth_last_frame_age_wall_s": depth_age,
                "rgb_stamp_advancing": self.rgb.stamp_is_advancing(
                    now, self.maximum_age
                ),
                "depth_stamp_advancing": self.depth.stamp_is_advancing(
                    now, self.maximum_age
                ),
                "rgb_stamp_advance_age_wall_s": self.rgb.stamp_advance_age(now),
                "depth_stamp_advance_age_wall_s": self.depth.stamp_advance_age(now),
                "rgb_stamp_repeats": self.rgb.stamp_repeats,
                "depth_stamp_repeats": self.depth.stamp_repeats,
                "rgb_stamp_regressions": self.rgb.regressions,
                "depth_stamp_regressions": self.depth.regressions,
            }
        return rgb, depth, info, values

    def _publish(self):
        rgb, depth, info, values = self._snapshot()
        level = DiagnosticStatus.OK
        messages = []
        if rgb is not None:
            values.update({
                "rgb_stamp": rgb.header.stamp.to_sec(),
                "rgb_encoding": rgb.encoding,
                "rgb_size": [int(rgb.width), int(rgb.height)],
                "rgb_frame": rgb.header.frame_id,
            })
        else:
            messages.append("RGB never received")
        if depth is not None:
            values.update({
                "depth_stamp": depth.header.stamp.to_sec(),
                "depth_encoding": depth.encoding,
                "depth_size": [int(depth.width), int(depth.height)],
                "depth_frame": depth.header.frame_id,
            })
        else:
            messages.append("depth never received")
        if info is not None:
            values.update({
                "camera_info_size": [int(info.width), int(info.height)],
                "camera_info_frame": info.header.frame_id,
            })
        else:
            messages.append("CameraInfo never received")

        stamp_delta = None
        if rgb is not None and depth is not None:
            stamp_delta = abs(
                (rgb.header.stamp - depth.header.stamp).to_sec()
            )
            values["rgb_depth_stamp_delta_s"] = stamp_delta
            if stamp_delta > self.maximum_stamp_delta:
                level = max(level, DiagnosticStatus.WARN)
                messages.append("RGB/depth timestamps diverged")

        for name in ("rgb", "depth"):
            age = values["{}_last_frame_age_wall_s".format(name)]
            rate = values["{}_wall_rate_hz".format(name)]
            if age > self.maximum_age:
                level = DiagnosticStatus.ERROR
                messages.append("{} stream stale".format(name))
            elif rate < self.minimum_rate:
                level = max(level, DiagnosticStatus.WARN)
                messages.append("{} wall rate low".format(name))
            if not values["{}_stamp_advancing".format(name)]:
                level = max(level, DiagnosticStatus.WARN)
                messages.append("{} timestamp not yet advancing".format(name))

        tf_ok = False
        tf_error = ""
        if rgb is not None and rgb.header.stamp != rospy.Time():
            frame = self.camera_frame or rgb.header.frame_id
            try:
                self.tf_buffer.lookup_transform(
                    self.map_frame, frame, rgb.header.stamp,
                    rospy.Duration(self.tf_timeout),
                )
                tf_ok = True
            except Exception as exc:
                tf_error = str(exc)
                level = max(level, DiagnosticStatus.WARN)
                messages.append("image-time TF unavailable")
        values["image_time_tf_ok"] = tf_ok
        values["image_time_tf_error"] = tf_error
        values["diagnostic_wall_time"] = time.time()

        status = DiagnosticStatus()
        status.name = "mine_camera/rgbd"
        status.hardware_id = "uav_downward_rgbd"
        status.level = level
        status.message = "; ".join(dict.fromkeys(messages)) or "RGB-D healthy"
        status.values = [
            self._value(key, value) for key, value in sorted(values.items())
        ]
        output = DiagnosticArray()
        output.header.stamp = rospy.Time.now()
        output.status = [status]
        self.publisher.publish(output)

    def _wall_loop(self):
        while not rospy.is_shutdown():
            try:
                self._publish()
            except Exception as exc:
                rospy.logerr_throttle(
                    2.0, "[MineCameraDiagnostics] publish failed: %s", exc
                )
            time.sleep(1.0)


def main():
    rospy.init_node("mine_camera_diagnostics")
    MineCameraDiagnostics()
    rospy.spin()


if __name__ == "__main__":
    main()
