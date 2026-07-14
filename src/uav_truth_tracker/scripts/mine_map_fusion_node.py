#!/usr/bin/env python3
"""Fuse frame-local UAV mine observations into a persistent map-frame mine map."""

import json
import math
import os
import threading
from collections import deque, namedtuple
from pathlib import Path

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String, UInt32MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

from uav_truth_tracker.msg import (
    MineDetectionArray,
    MineMap,
    MineMapEntry,
)


UgvOdomSample = namedtuple("UgvOdomSample", ("stamp", "x", "y", "z"))


class MineTrack:
    def __init__(self, track_id, detection, stamp):
        self.id = int(track_id)
        self.position = np.asarray(
            [
                detection.position.x,
                detection.position.y,
                detection.position.z,
            ],
            dtype=np.float64,
        )
        self.weight_sum = max(float(detection.confidence), 0.05)
        self.max_confidence = float(detection.confidence)
        self.min_confidence = float(detection.confidence)
        self.confidence_sum = float(detection.confidence)
        self.observation_count = 1
        self.first_seen = stamp
        self.last_seen = stamp
        self.confirmed = False
        self.confirmation_mode = ""
        self.mean_xy = self.position[:2].copy()
        self.m2_xy = np.zeros(2, dtype=np.float64)
        # 终生 Welford 统计用于诊断；有界历史用于判断“早期抖动后
        # 最近已经稳定”。不使用 Gazebo 真值。
        self.observation_history = deque(maxlen=64)
        self.observation_history.append(
            (float(stamp.to_sec()), self.position.copy(), float(detection.confidence))
        )

    def update(self, detection, stamp, min_interval):
        dt = (stamp - self.last_seen).to_sec()
        if dt < min_interval:
            return False
        point = np.asarray(
            [
                detection.position.x,
                detection.position.y,
                detection.position.z,
            ],
            dtype=np.float64,
        )
        confidence = float(detection.confidence)
        weight = max(confidence, 0.05)
        self.position = (
            self.position * self.weight_sum + point * weight
        ) / (self.weight_sum + weight)
        self.weight_sum += weight
        self.max_confidence = max(self.max_confidence, confidence)
        self.min_confidence = min(self.min_confidence, confidence)
        self.confidence_sum += confidence

        old_count = self.observation_count
        self.observation_count += 1
        delta = point[:2] - self.mean_xy
        self.mean_xy += delta / self.observation_count
        self.m2_xy += delta * (point[:2] - self.mean_xy)
        self.last_seen = stamp
        self.observation_history.append(
            (float(stamp.to_sec()), point.copy(), confidence)
        )
        return self.observation_count != old_count

    def xy_std(self):
        if self.observation_count < 2:
            return 0.0
        variance = self.m2_xy / max(self.observation_count - 1, 1)
        return float(math.sqrt(max(float(np.max(variance)), 0.0)))

    def mean_confidence(self):
        return self.confidence_sum / max(self.observation_count, 1)

    def observation_span(self):
        return max((self.last_seen - self.first_seen).to_sec(), 0.0)

    def robust_cluster(self, window_size=12, inlier_radius=0.12):
        """返回最近观测中围绕中位数中心的稳定内点簇。"""
        history = list(self.observation_history)[-max(int(window_size), 1):]
        if not history:
            return {
                "count": 0, "position": self.position.copy(),
                "xy_std": math.inf, "span": 0.0, "min_confidence": 0.0,
            }
        stamps = np.asarray([item[0] for item in history], dtype=np.float64)
        points = np.asarray([item[1] for item in history], dtype=np.float64)
        confidences = np.asarray([item[2] for item in history], dtype=np.float64)
        center = np.median(points, axis=0)
        distance = np.linalg.norm(points[:, :2] - center[:2], axis=1)
        inliers = distance <= max(float(inlier_radius), 0.001)
        if not np.any(inliers):
            inliers[int(np.argmin(distance))] = True
        # 再以首轮内点中位数收敛一次，抑制窗口边缘的旧抖动。
        center = np.median(points[inliers], axis=0)
        distance = np.linalg.norm(points[:, :2] - center[:2], axis=1)
        inliers = distance <= max(float(inlier_radius), 0.001)
        selected = points[inliers]
        selected_stamps = stamps[inliers]
        selected_confidences = confidences[inliers]
        if selected.shape[0] < 2:
            xy_std = 0.0
        else:
            xy_std = float(math.sqrt(max(
                float(np.max(np.var(selected[:, :2], axis=0, ddof=1))), 0.0
            )))
        return {
            "count": int(selected.shape[0]),
            "position": np.median(selected, axis=0),
            "xy_std": xy_std,
            "span": float(max(selected_stamps.max() - selected_stamps.min(), 0.0)),
            "min_confidence": float(selected_confidences.min()),
        }

    def to_entry(self):
        entry = MineMapEntry()
        entry.id = self.id
        entry.position.x = float(self.position[0])
        entry.position.y = float(self.position[1])
        entry.position.z = float(self.position[2])
        entry.confidence = float(self.max_confidence)
        entry.observation_count = self.observation_count
        entry.first_seen = self.first_seen
        entry.last_seen = self.last_seen
        entry.confirmed = self.confirmed
        return entry


class MineMapFusion:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/mine_detection/raw")
        self.map_topic = rospy.get_param("~map_topic", "/mine_detection/map")
        self.confirmed_topic = rospy.get_param(
            "~confirmed_topic", "/mine_detection/confirmed"
        )
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/mine_detection/markers"
        )
        self.json_topic = rospy.get_param(
            "~json_topic", "/mine_detection/map_json"
        )
        self.suppressed_ids_topic = rospy.get_param(
            "~suppressed_ids_topic",
            "/mine_mission/suppressed_detection_ids",
        )
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.output_file = Path(
            os.path.abspath(
                os.path.expanduser(
                    rospy.get_param("~output_file", "~/.ros/mine_map.yaml")
                )
            )
        )
        self.model_path = os.path.abspath(
            os.path.expanduser(rospy.get_param("~model_path", ""))
        )

        self.association_radius = max(
            float(rospy.get_param("~association_radius", 0.30)), 0.02
        )
        self.association_height_tolerance = max(
            float(rospy.get_param("~association_height_tolerance", 0.25)),
            0.05,
        )
        self.confirmed_update_radius = min(
            max(float(rospy.get_param("~confirmed_update_radius", 0.12)), 0.03),
            self.association_radius,
        )
        # Keep confirmed position updates tight (0.12 m), but use a wider,
        # identity-only quarantine around every physical mine.  Depth/TF
        # outliers in the recorded run formed fragments 0.30..0.88 m from the
        # same mine; capping this gate at association_radius created the yellow
        # clouds and even a second confirmed ID.  This wider gate never moves a
        # confirmed position and remains well below the mission's mine spacing.
        self.duplicate_suppression_radius = min(
            max(
                float(rospy.get_param("~duplicate_suppression_radius", 1.00)),
                self.confirmed_update_radius,
            ),
            1.00,
        )
        self.min_confirmations = max(
            int(rospy.get_param("~min_confirmations", 3)), 3
        )
        self.min_observation_interval = max(
            float(rospy.get_param("~min_observation_interval", 0.20)), 0.0
        )
        self.max_confirmation_std = max(
            float(rospy.get_param("~max_confirmation_std", 0.10)), 0.01
        )
        # The normal path remains a three-frame confirmation.  A clean fly-by
        # may use two independent frames only when confidence, spatial spread
        # and timestamp span all satisfy the tighter fast gate.  A single YOLO
        # frame is never enough to create a red/dispatchable mine.
        self.fast_confirmation_count = max(
            int(rospy.get_param("~fast_confirmation_count", 2)), 2
        )
        self.fast_confirmation_confidence = min(
            max(
                float(rospy.get_param("~fast_confirmation_confidence", 0.75)),
                0.01,
            ),
            0.99,
        )
        self.fast_confirmation_std = max(
            float(rospy.get_param("~fast_confirmation_std", 0.06)), 0.005
        )
        self.fast_confirmation_min_span = max(
            float(rospy.get_param("~fast_confirmation_min_span", 0.15)), 0.0
        )
        # 正常门保留终生方差约束。对早期有误差、但后续多帧已稳定的
        # 真实地雷，用最近稳定簇给出另一条严格的多帧确认路径。
        self.persistent_confirmation_count = max(
            int(rospy.get_param("~persistent_confirmation_count", 6)), 4
        )
        self.confirmation_window_size = max(
            int(rospy.get_param("~confirmation_window_size", 12)),
            self.persistent_confirmation_count,
        )
        self.persistent_confirmation_radius = max(
            float(rospy.get_param("~persistent_confirmation_radius", 0.12)), 0.02
        )
        self.persistent_confirmation_std = max(
            float(rospy.get_param("~persistent_confirmation_std", 0.08)), 0.01
        )
        self.persistent_confirmation_min_span = max(
            float(rospy.get_param("~persistent_confirmation_min_span", 0.80)), 0.0
        )
        self.candidate_timeout = max(
            float(rospy.get_param("~candidate_timeout", 300.0)), 0.5
        )
        # Once the UGV reaches a mine, its yellow chassis/arm parts can look
        # like a mine to the UAV segmenter.  Reject observations in the UGV's
        # timestamped footprint before track association so they can neither
        # create a new mine nor move an existing one.  A wider gate only
        # rejects points clearly above the base, preserving real ground mines.
        self.ugv_exclusion_enabled = bool(
            rospy.get_param("~ugv_exclusion_enabled", True)
        )
        self.ugv_odom_topic = rospy.get_param(
            "~ugv_odom_topic", "/odometry/filtered_map"
        )
        self.ugv_exclusion_radius = max(
            float(rospy.get_param("~ugv_exclusion_radius", 1.50)), 0.0
        )
        self.ugv_high_object_radius = max(
            float(rospy.get_param("~ugv_high_object_radius", 3.00)),
            self.ugv_exclusion_radius,
        )
        self.ugv_high_object_min_height = max(
            float(rospy.get_param("~ugv_high_object_min_height", 0.50)), 0.05
        )
        self.ugv_odom_match_tolerance = max(
            float(rospy.get_param("~ugv_odom_match_tolerance", 0.75)), 0.05
        )
        self.ugv_odom_history_duration = max(
            float(rospy.get_param("~ugv_odom_history_duration", 10.0)),
            2.0 * self.ugv_odom_match_tolerance,
        )
        # Optional absolute sanity gate only. Terrain validity is checked by the
        # localizer against a depth ring around each mask; using a narrow world-z
        # interval here would discard every mine on a hill.
        self.min_map_z = float(rospy.get_param("~min_map_z", -1.0e6))
        self.max_map_z = float(rospy.get_param("~max_map_z", 1.0e6))
        if self.min_map_z >= self.max_map_z:
            raise ValueError("min_map_z must be smaller than max_map_z")
        self.save_period = max(float(rospy.get_param("~save_period", 1.0)), 0.1)
        self.reset_on_start = bool(rospy.get_param("~reset_on_start", True))

        self.tracks = {}
        self.next_id = 1
        self.revision = 0
        self.last_saved_revision = -1
        self.last_save_time = rospy.Time(0)
        self.lock = threading.RLock()
        self.ugv_odom_history = deque(maxlen=1200)
        self.ugv_filter_status = (
            "waiting_odometry" if self.ugv_exclusion_enabled else "disabled"
        )
        self.ugv_rejected_near_total = 0
        self.ugv_rejected_high_total = 0
        self.ugv_filter_bypass_total = 0
        self.suppressed_track_ids = set()

        self.map_pub = rospy.Publisher(
            self.map_topic, MineMap, queue_size=2, latch=True
        )
        self.confirmed_pub = rospy.Publisher(
            self.confirmed_topic, PoseArray, queue_size=2, latch=True
        )
        self.marker_pub = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=2, latch=True
        )
        self.json_pub = rospy.Publisher(
            self.json_topic, String, queue_size=2, latch=True
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, MineDetectionArray, self._detections_cb, queue_size=5
        )
        self.ugv_odom_subscriber = rospy.Subscriber(
            self.ugv_odom_topic, Odometry, self._ugv_odom_cb, queue_size=100
        )
        self.suppressed_ids_subscriber = rospy.Subscriber(
            self.suppressed_ids_topic,
            UInt32MultiArray,
            self._suppressed_ids_cb,
            queue_size=1,
        )
        self.clear_service = rospy.Service(
            "/mine_detection/clear_map", Trigger, self._clear_cb
        )
        self.save_service = rospy.Service(
            "/mine_detection/save_map", Trigger, self._save_cb
        )
        self.timer = rospy.Timer(rospy.Duration(0.5), self._timer_cb)

        # Subscribers and the prune timer are already live at this point. Use
        # the callback lock so the first detection cannot race the latched
        # initial map or its on-disk snapshot.
        with self.lock:
            if self.reset_on_start:
                self._save(force=True)
            self._publish()
        rospy.on_shutdown(self._shutdown_cb)
        rospy.loginfo(
            "[MineMapFusion] input=%s map=%s confirmed=%s radius=%.2fm "
            "confirmed_radius=%.2fm duplicate_radius=%.2fm z_gate=%.2fm "
            "confirmations=%d max_std=%.2fm fast=%dx(conf>=%.2f,std<=%.2fm,"
            "span>=%.2fs) persistent=%d/%d(radius<=%.2fm,std<=%.2fm,span>=%.2fs) "
            "candidate_timeout=%.1fs UGV_filter=%s topic=%s "
            "near=%.2fm high=(%.2fm,+%.2fm) odom_dt<=%.2fs file=%s",
            self.input_topic,
            self.map_topic,
            self.confirmed_topic,
            self.association_radius,
            self.confirmed_update_radius,
            self.duplicate_suppression_radius,
            self.association_height_tolerance,
            self.min_confirmations,
            self.max_confirmation_std,
            self.fast_confirmation_count,
            self.fast_confirmation_confidence,
            self.fast_confirmation_std,
            self.fast_confirmation_min_span,
            self.persistent_confirmation_count,
            self.confirmation_window_size,
            self.persistent_confirmation_radius,
            self.persistent_confirmation_std,
            self.persistent_confirmation_min_span,
            self.candidate_timeout,
            self.ugv_exclusion_enabled,
            self.ugv_odom_topic,
            self.ugv_exclusion_radius,
            self.ugv_high_object_radius,
            self.ugv_high_object_min_height,
            self.ugv_odom_match_tolerance,
            self.output_file,
        )

    def _ugv_odom_cb(self, msg):
        """Keep map-frame UGV poses long enough to match delayed inference."""
        frame_id = str(msg.header.frame_id or "").lstrip("/")
        expected_frame = str(self.map_frame or "").lstrip("/")
        if frame_id != expected_frame:
            rospy.logwarn_throttle(
                2.0,
                "[MineMapFusionUGV] ignoring odometry in frame '%s'; "
                "expected '%s'",
                msg.header.frame_id or "<empty>",
                self.map_frame,
            )
            return

        stamp = msg.header.stamp
        if stamp == rospy.Time():
            stamp = rospy.Time.now()
            rospy.logwarn_throttle(
                5.0,
                "[MineMapFusionUGV] odometry has zero stamp; using current "
                "ROS time",
            )
        values = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(
                2.0, "[MineMapFusionUGV] ignoring non-finite UGV odometry"
            )
            return

        sample = UgvOdomSample(stamp, *map(float, values))
        with self.lock:
            # Gazebo can reset /clock between full runs.  Do not let samples
            # from the previous epoch become the nearest timestamp.
            if (self.ugv_odom_history
                    and (self.ugv_odom_history[-1].stamp - stamp).to_sec()
                    > self.ugv_odom_history_duration):
                self.ugv_odom_history.clear()
            self.ugv_odom_history.append(sample)
            newest = max(item.stamp for item in self.ugv_odom_history)
            # rospy.Time rejects negative values during the first few seconds
            # of simulated time.
            cutoff = rospy.Time.from_sec(max(
                newest.to_sec() - self.ugv_odom_history_duration, 0.0
            ))
            retained = [
                item for item in self.ugv_odom_history
                if item.stamp >= cutoff
            ]
            if len(retained) != len(self.ugv_odom_history):
                self.ugv_odom_history.clear()
                self.ugv_odom_history.extend(retained)

    def _suppressed_ids_cb(self, msg):
        incoming = {int(value) for value in msg.data if int(value) > 0}
        with self.lock:
            if incoming == self.suppressed_track_ids:
                return
            self.suppressed_track_ids = incoming
            # The internal frozen track remains available for duplicate
            # quarantine, but every public map/RViz representation changes.
            self.revision += 1
            self._publish()
            self._save(force=True)
        rospy.logwarn(
            "[MineMapFusion] mission suppressed picked source IDs: %s",
            ",".join("M{:03d}".format(value) for value in sorted(incoming))
            or "none",
        )

    def _ugv_sample_for_stamp(self, stamp):
        if not self.ugv_exclusion_enabled:
            return None, "disabled", None
        if not self.ugv_odom_history:
            return None, "no_odometry", None
        sample = min(
            self.ugv_odom_history,
            key=lambda item: abs((item.stamp - stamp).to_sec()),
        )
        stamp_delta = abs((sample.stamp - stamp).to_sec())
        if stamp_delta > self.ugv_odom_match_tolerance:
            return None, "stamp_mismatch", stamp_delta
        return sample, "active", stamp_delta

    def _exclude_ugv_observations(self, detections, stamp):
        """Return map detections not attributable to the UGV itself."""
        detections = list(detections)
        sample, status, stamp_delta = self._ugv_sample_for_stamp(stamp)
        if sample is None:
            self.ugv_filter_status = status
            if status not in ("disabled",):
                self.ugv_filter_bypass_total += 1
                detail = (
                    "no map-frame odometry received"
                    if status == "no_odometry"
                    else "nearest odometry differs by {:.3f}s".format(stamp_delta)
                )
                rospy.logwarn_throttle(
                    5.0,
                    "[MineMapFusionUGV] dynamic exclusion bypassed: %s; "
                    "far-field survey detections remain eligible",
                    detail,
                )
            return detections

        kept = []
        rejected_near = 0
        rejected_high = 0
        for detection in detections:
            xy_distance = math.hypot(
                float(detection.position.x) - sample.x,
                float(detection.position.y) - sample.y,
            )
            height_above_base = float(detection.position.z) - sample.z
            if xy_distance <= self.ugv_exclusion_radius:
                rejected_near += 1
                continue
            if (xy_distance <= self.ugv_high_object_radius
                    and height_above_base >= self.ugv_high_object_min_height):
                rejected_high += 1
                continue
            kept.append(detection)

        self.ugv_filter_status = "active"
        self.ugv_rejected_near_total += rejected_near
        self.ugv_rejected_high_total += rejected_high
        if rejected_near or rejected_high:
            rospy.logwarn_throttle(
                1.0,
                "[MineMapFusionUGV] rejected %d near-body and %d elevated "
                "detections using UGV map pose (%.2f, %.2f, %.2f), "
                "stamp delta %.3fs",
                rejected_near,
                rejected_high,
                sample.x,
                sample.y,
                sample.z,
                stamp_delta,
            )
        return kept

    @staticmethod
    def _xy_distance(track, detection):
        return math.hypot(
            float(track.position[0]) - detection.position.x,
            float(track.position[1]) - detection.position.y,
        )

    def _associate(self, detections):
        pairs = []
        track_ids = sorted(self.tracks)
        for track_id in track_ids:
            track = self.tracks[track_id]
            for detection_index, detection in enumerate(detections):
                distance = self._xy_distance(track, detection)
                xy_limit = (
                    self.confirmed_update_radius
                    if track.confirmed else self.association_radius
                )
                height_error = abs(
                    float(track.position[2]) - float(detection.position.z)
                )
                if (distance <= xy_limit
                        and height_error <= self.association_height_tolerance):
                    pairs.append((distance, track_id, detection_index))
        pairs.sort(key=lambda item: (item[0], item[1], item[2]))

        assignments = []
        assigned_tracks = set()
        assigned_detections = set()
        for distance, track_id, detection_index in pairs:
            if track_id in assigned_tracks or detection_index in assigned_detections:
                continue
            assignments.append((track_id, detection_index, distance))
            assigned_tracks.add(track_id)
            assigned_detections.add(detection_index)
        return assignments, assigned_detections

    def _duplicate_track_id(self, detection, radius=None):
        """Return an existing physical identity near an unmatched detection."""
        limit = (
            self.duplicate_suppression_radius
            if radius is None else float(radius)
        )
        matches = []
        for track_id, track in self.tracks.items():
            distance = self._xy_distance(track, detection)
            height_error = abs(
                float(track.position[2]) - float(detection.position.z)
            )
            if (distance <= limit
                    and height_error <= self.association_height_tolerance):
                matches.append((distance, int(track_id)))
        return min(matches)[1] if matches else None

    @staticmethod
    def _preferred_duplicate_track(first, second):
        if first.confirmed != second.confirmed:
            return first if first.confirmed else second
        if first.confirmed:
            # Preserve a confirmed public ID once it has been dispatched.
            return first if first.id < second.id else second
        if first.observation_count != second.observation_count:
            return (
                first if first.observation_count > second.observation_count
                else second
            )
        return first if first.id < second.id else second

    def _suppress_duplicate_tracks(self):
        """Collapse same-ground track fragments before publishing tasks."""
        removed = []
        while True:
            duplicate_pair = None
            track_ids = sorted(self.tracks)
            for offset, first_id in enumerate(track_ids):
                first = self.tracks[first_id]
                for second_id in track_ids[offset + 1:]:
                    second = self.tracks[second_id]
                    distance = math.hypot(
                        float(first.position[0] - second.position[0]),
                        float(first.position[1] - second.position[1]),
                    )
                    height_error = abs(
                        float(first.position[2] - second.position[2])
                    )
                    if (distance <= self.duplicate_suppression_radius
                            and height_error <= self.association_height_tolerance):
                        duplicate_pair = (first, second, distance, height_error)
                        break
                if duplicate_pair is not None:
                    break
            if duplicate_pair is None:
                break
            first, second, distance, height_error = duplicate_pair
            winner = self._preferred_duplicate_track(first, second)
            loser = second if winner is first else first
            del self.tracks[loser.id]
            removed.append(loser.id)
            rospy.logwarn(
                "[MineMapFusion] suppressed duplicate M%03d near M%03d "
                "(xy=%.3fm z=%.3fm)",
                loser.id, winner.id, distance, height_error,
            )
        return removed

    def _confirm_ready_tracks(self):
        newly_confirmed = []
        for track in self.tracks.values():
            if track.confirmed:
                continue
            robust = track.robust_cluster(
                getattr(self, "confirmation_window_size", 12),
                getattr(self, "persistent_confirmation_radius", 0.12),
            )
            normal_ready = (
                track.observation_count >= self.min_confirmations
                and track.xy_std() <= self.max_confirmation_std
            )
            fast_ready = (
                track.observation_count >= self.fast_confirmation_count
                and track.min_confidence >= self.fast_confirmation_confidence
                and track.xy_std() <= self.fast_confirmation_std
                and track.observation_span() >= self.fast_confirmation_min_span
            )
            persistent_ready = (
                robust["count"] >= getattr(
                    self, "persistent_confirmation_count", 6
                )
                and robust["xy_std"] <= getattr(
                    self, "persistent_confirmation_std", 0.08
                )
                and robust["span"] >= getattr(
                    self, "persistent_confirmation_min_span", 0.80
                )
            )
            if normal_ready or fast_ready or persistent_ready:
                track.confirmed = True
                if persistent_ready and not (normal_ready or fast_ready):
                    # 确认位置必须来自稳定内点簇，不继续使用被早期抖动
                    # 拉偏的终生加权均值。
                    track.position = np.asarray(robust["position"], dtype=np.float64)
                    track.confirmation_mode = "persistent_robust"
                elif fast_ready and not normal_ready:
                    track.confirmation_mode = "fast_high_confidence"
                else:
                    track.confirmation_mode = "normal_multiframe"
                newly_confirmed.append(track)
        return newly_confirmed

    def _detections_cb(self, msg):
        stamp = msg.header.stamp
        if stamp == rospy.Time():
            stamp = rospy.Time.now()
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn_throttle(
                2.0,
                "[MineMapFusion] rejecting detections in frame %s; expected %s",
                msg.header.frame_id,
                self.map_frame,
            )
            return

        finite_detections = [
            detection
            for detection in msg.detections
            if all(
                math.isfinite(value)
                for value in (
                    detection.position.x,
                    detection.position.y,
                    detection.position.z,
                    detection.confidence,
                )
            )
            and self.min_map_z <= detection.position.z <= self.max_map_z
        ]
        rejected_height = len(msg.detections) - len(finite_detections)
        if rejected_height:
            rospy.logwarn_throttle(
                2.0,
                "[MineMapFusion] rejected %d non-finite/out-of-height detections "
                "(allowed map z %.2f..%.2f m)",
                rejected_height,
                self.min_map_z,
                self.max_map_z,
            )
        with self.lock:
            # This must precede both association and track creation.  Merely
            # blocking new tracks would still allow a robot-coloured mask to
            # drag a nearby candidate/confirmed mine.
            finite_detections = self._exclude_ugv_observations(
                finite_detections, stamp
            )
            self._prune(stamp)
            assignments, assigned_detections = self._associate(finite_detections)
            changed = False
            for track_id, detection_index, _distance in assignments:
                changed = self.tracks[track_id].update(
                    finite_detections[detection_index],
                    stamp,
                    self.min_observation_interval,
                ) or changed
            for detection_index, detection in enumerate(finite_detections):
                if detection_index in assigned_detections:
                    continue
                duplicate_id = self._duplicate_track_id(detection)
                if duplicate_id is not None:
                    rospy.loginfo_throttle(
                        1.0,
                        "[MineMapFusion] quarantined unmatched observation near "
                        "M%03d instead of creating a duplicate track",
                        duplicate_id,
                    )
                    continue
                track = MineTrack(self.next_id, detection, stamp)
                self.tracks[track.id] = track
                self.next_id += 1
                changed = True

            newly_confirmed = self._confirm_ready_tracks()
            removed_duplicates = self._suppress_duplicate_tracks()
            if removed_duplicates:
                changed = True
                newly_confirmed = [
                    track for track in newly_confirmed
                    if track.id in self.tracks
                ]
            if newly_confirmed:
                changed = True
                for track in newly_confirmed:
                    robust = track.robust_cluster(
                        self.confirmation_window_size,
                        self.persistent_confirmation_radius,
                    )
                    rospy.logwarn(
                        "[MineMapFusion] CONFIRMED M%03d map=(%.3f, %.3f, %.3f) "
                        "conf=%.3f observations=%d lifetime_std=%.3fm "
                        "recent=%d recent_std=%.3fm mode=%s",
                        track.id,
                        track.position[0],
                        track.position[1],
                        track.position[2],
                        track.max_confidence,
                        track.observation_count,
                        track.xy_std(),
                        robust["count"],
                        robust["xy_std"],
                        track.confirmation_mode,
                    )
            if changed:
                self.revision += 1
            self._publish(stamp)
            self._save()

    def _prune(self, now):
        removed = []
        for track_id, track in self.tracks.items():
            if track.confirmed:
                continue
            age = (now - track.last_seen).to_sec()
            if age > self.candidate_timeout:
                removed.append(track_id)
        for track_id in removed:
            del self.tracks[track_id]
        if removed:
            self.revision += 1
        return bool(removed)

    def _map_message(self, stamp):
        msg = MineMap()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.revision = self.revision
        msg.mines = [
            self.tracks[track_id].to_entry() for track_id in sorted(self.tracks)
            if track_id not in self.suppressed_track_ids
        ]
        return msg

    def _confirmed_pose_array(self, stamp):
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        for track_id in sorted(self.tracks):
            if track_id in self.suppressed_track_ids:
                continue
            track = self.tracks[track_id]
            if not track.confirmed:
                continue
            pose = Pose()
            pose.position.x = float(track.position[0])
            pose.position.y = float(track.position[1])
            pose.position.z = float(track.position[2])
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def _markers(self, stamp):
        markers = []
        delete_all = Marker()
        delete_all.header.stamp = stamp
        delete_all.header.frame_id = self.map_frame
        delete_all.action = Marker.DELETEALL
        markers.append(delete_all)
        for track_id in sorted(self.tracks):
            if track_id in self.suppressed_track_ids:
                continue
            track = self.tracks[track_id]
            sphere = Marker()
            sphere.header.stamp = stamp
            sphere.header.frame_id = self.map_frame
            sphere.ns = "confirmed_mines" if track.confirmed else "mine_candidates"
            sphere.id = track.id * 2
            sphere.type = Marker.CYLINDER if track.confirmed else Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(track.position[0])
            sphere.pose.position.y = float(track.position[1])
            sphere.pose.position.z = float(track.position[2]) + 0.14
            sphere.pose.orientation.w = 1.0
            if track.confirmed:
                sphere.scale.x = sphere.scale.y = 0.38
                sphere.scale.z = 0.18
                sphere.color.r = 1.0
                sphere.color.g = 0.05
                sphere.color.b = 0.02
                sphere.color.a = 0.95
            else:
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.22
                sphere.color.r = 1.0
                sphere.color.g = 0.75
                sphere.color.b = 0.0
                sphere.color.a = 0.85
            markers.append(sphere)

            text = Marker()
            text.header = sphere.header
            text.ns = sphere.ns + "_labels"
            text.id = track.id * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(track.position[0])
            text.pose.position.y = float(track.position[1])
            text.pose.position.z = sphere.pose.position.z + 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.26
            text.color.r = sphere.color.r
            text.color.g = sphere.color.g
            text.color.b = sphere.color.b
            text.color.a = 1.0
            robust = track.robust_cluster(
                getattr(self, "confirmation_window_size", 12),
                getattr(self, "persistent_confirmation_radius", 0.12),
            )
            mode_label = {
                "fast_high_confidence": "FAST",
                "persistent_robust": "ROBUST",
                "normal_multiframe": "MULTI",
            }.get(track.confirmation_mode, "MULTI")
            state = "CONF/{}".format(mode_label) if track.confirmed else "CAND"
            text.text = (
                "M{:03d} {} ({:.2f},{:.2f}) c={:.2f} n={}/{} "
                "std={:.3f} recent={}/{:.3f}"
            ).format(
                track.id,
                state,
                track.position[0],
                track.position[1],
                track.max_confidence,
                track.observation_count,
                self.min_confirmations,
                track.xy_std(),
                robust["count"],
                robust["xy_std"],
            )
            markers.append(text)
        return MarkerArray(markers=markers)

    def _serializable(self):
        confirmed = []
        candidates = []
        for track_id in sorted(self.tracks):
            if track_id in self.suppressed_track_ids:
                continue
            track = self.tracks[track_id]
            robust = track.robust_cluster(
                getattr(self, "confirmation_window_size", 12),
                getattr(self, "persistent_confirmation_radius", 0.12),
            )
            item = {
                "id": int(track.id),
                "position": {
                    "x": float(track.position[0]),
                    "y": float(track.position[1]),
                    "z": float(track.position[2]),
                },
                "confidence": float(track.max_confidence),
                "observation_count": int(track.observation_count),
                "position_std_xy": float(track.xy_std()),
                "recent_inlier_count": int(robust["count"]),
                "recent_position_std_xy": float(robust["xy_std"]),
                "recent_observation_span": float(robust["span"]),
                "mean_confidence": float(track.mean_confidence()),
                "minimum_confidence": float(track.min_confidence),
                "observation_span": float(track.observation_span()),
                "confirmation_mode": getattr(track, "confirmation_mode", ""),
                "first_seen": float(track.first_seen.to_sec()),
                "last_seen": float(track.last_seen.to_sec()),
            }
            (confirmed if track.confirmed else candidates).append(item)
        return {
            "format": "uav_landmine_map_v1",
            "frame_id": self.map_frame,
            "revision": int(self.revision),
            "model_path": self.model_path,
            "confirmed_count": len(confirmed),
            "candidate_count": len(candidates),
            "confirmed_mines": confirmed,
            "candidates": candidates,
            "ugv_exclusion": {
                "enabled": bool(self.ugv_exclusion_enabled),
                "odometry_topic": self.ugv_odom_topic,
                "status": self.ugv_filter_status,
                "rejected_near_total": int(self.ugv_rejected_near_total),
                "rejected_high_total": int(self.ugv_rejected_high_total),
                "bypass_total": int(self.ugv_filter_bypass_total),
            },
            "suppressed_track_ids": sorted(self.suppressed_track_ids),
        }

    def _publish(self, stamp=None):
        stamp = stamp or rospy.Time.now()
        map_msg = self._map_message(stamp)
        self.map_pub.publish(map_msg)
        self.confirmed_pub.publish(self._confirmed_pose_array(stamp))
        self.marker_pub.publish(self._markers(stamp))
        payload = self._serializable()
        self.json_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        rospy.loginfo_throttle(
            1.0,
            "[MineMapFusionDiag] revision=%d candidates=%d confirmed=%d "
            "UGV_filter=%s rejected=(near:%d high:%d) bypass=%d file=%s",
            self.revision,
            payload["candidate_count"],
            payload["confirmed_count"],
            self.ugv_filter_status,
            self.ugv_rejected_near_total,
            self.ugv_rejected_high_total,
            self.ugv_filter_bypass_total,
            self.output_file,
        )

    def _save(self, force=False):
        now = rospy.Time.now()
        if not force:
            if self.last_saved_revision == self.revision:
                return False
            if (
                self.last_save_time != rospy.Time(0)
                and (now - self.last_save_time).to_sec() < self.save_period
            ):
                return False
        payload = self._serializable()
        try:
            self.output_file.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.output_file.with_suffix(self.output_file.suffix + ".tmp")
            with temporary.open("w", encoding="utf-8") as stream:
                yaml.safe_dump(payload, stream, sort_keys=False, allow_unicode=True)
            os.replace(str(temporary), str(self.output_file))
            self.last_saved_revision = self.revision
            self.last_save_time = now
            return True
        except Exception as exc:
            rospy.logerr_throttle(
                2.0, "[MineMapFusion] failed to save %s: %s", self.output_file, exc
            )
            return False

    def _timer_cb(self, _event):
        with self.lock:
            changed = self._prune(rospy.Time.now())
            if changed:
                self._publish()
            self._save()

    def _clear_cb(self, _request):
        with self.lock:
            self.tracks.clear()
            self.next_id = 1
            self.revision += 1
            self._publish()
            saved = self._save(force=True)
        return TriggerResponse(
            success=saved,
            message="mine map cleared{}".format(" and saved" if saved else ""),
        )

    def _save_cb(self, _request):
        with self.lock:
            saved = self._save(force=True)
        return TriggerResponse(
            success=saved,
            message="saved {}".format(self.output_file) if saved else "save failed",
        )

    def _shutdown_cb(self):
        # Shutdown hooks may overlap a subscriber or timer callback. Serialize
        # the final atomic snapshot with all track mutations.
        with self.lock:
            self._save(force=True)


def main():
    rospy.init_node("mine_map_fusion")
    MineMapFusion()
    rospy.spin()


if __name__ == "__main__":
    main()
