#!/usr/bin/env python3
"""Regression tests for persistent UAV mine-map confirmation."""

import importlib.util
import threading
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import rospy


PACKAGE = Path(__file__).resolve().parents[1]


def load_fusion_module():
    path = PACKAGE / "scripts" / "mine_map_fusion_node.py"
    spec = importlib.util.spec_from_file_location("mine_map_fusion_node", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


FUSION_MODULE = load_fusion_module()
MineTrack = FUSION_MODULE.MineTrack
MineMapFusion = FUSION_MODULE.MineMapFusion
UgvOdomSample = FUSION_MODULE.UgvOdomSample


def detection(x, y, confidence, z=0.04):
    return SimpleNamespace(
        position=SimpleNamespace(x=float(x), y=float(y), z=float(z)),
        confidence=float(confidence),
    )


def make_fusion(track):
    fusion = MineMapFusion.__new__(MineMapFusion)
    fusion.tracks = {track.id: track}
    fusion.min_confirmations = 3
    fusion.max_confirmation_std = 0.20
    fusion.fast_confirmation_count = 3
    fusion.fast_confirmation_confidence = 0.72
    fusion.fast_confirmation_std = 0.08
    fusion.fast_confirmation_min_span = 0.40
    fusion.candidate_timeout = 30.0
    fusion.association_radius = 0.30
    fusion.association_height_tolerance = 0.25
    fusion.confirmed_update_radius = 0.12
    fusion.duplicate_suppression_radius = 0.25
    fusion.map_frame = "map"
    fusion.ugv_exclusion_enabled = True
    fusion.ugv_odom_topic = "/odometry/filtered_map"
    fusion.ugv_exclusion_radius = 1.50
    fusion.ugv_high_object_radius = 3.00
    fusion.ugv_high_object_min_height = 0.50
    fusion.ugv_odom_match_tolerance = 0.75
    fusion.ugv_odom_history_duration = 10.0
    fusion.ugv_odom_history = deque(maxlen=1200)
    fusion.ugv_filter_status = "waiting_odometry"
    fusion.ugv_rejected_near_total = 0
    fusion.ugv_rejected_high_total = 0
    fusion.ugv_filter_bypass_total = 0
    fusion.lock = threading.RLock()
    fusion.revision = 0
    return fusion


def ugv_sample(stamp, x=0.0, y=0.0, z=0.20):
    return UgvOdomSample(rospy.Time.from_sec(stamp), x, y, z)


def odometry(stamp, frame="map", x=0.0, y=0.0, z=0.20):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=rospy.Time.from_sec(stamp), frame_id=frame
        ),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z)
            )
        ),
    )


class MineMapFusionLogicTest(unittest.TestCase):
    def setUp(self):
        # Throttled rospy logging consults ROS time; these are deliberately
        # master-free unit tests, so replace only the logger side effect.
        self.logwarn_throttle = mock.patch.object(
            FUSION_MODULE.rospy, "logwarn_throttle"
        ).start()
        self.addCleanup(mock.patch.stopall)

    def test_three_independent_high_confidence_stable_observations_confirm(self):
        track = MineTrack(1, detection(2.300, -1.100, 0.83), rospy.Time.from_sec(1.0))
        self.assertTrue(track.update(
            detection(2.304, -1.104, 0.78), rospy.Time.from_sec(1.6), 0.20
        ))
        self.assertTrue(track.update(
            detection(2.302, -1.102, 0.81), rospy.Time.from_sec(2.2), 0.20
        ))
        fusion = make_fusion(track)

        confirmed = fusion._confirm_ready_tracks()

        self.assertEqual(confirmed, [track])
        self.assertTrue(track.confirmed)
        self.assertEqual(track.confirmation_mode, "normal_multiframe")

    def test_low_confidence_pair_remains_candidate_until_third_frame(self):
        track = MineTrack(2, detection(1.0, 1.0, 0.68), rospy.Time.from_sec(1.0))
        track.update(detection(1.01, 1.0, 0.80), rospy.Time.from_sec(1.6), 0.20)
        fusion = make_fusion(track)

        self.assertEqual(fusion._confirm_ready_tracks(), [])
        self.assertFalse(track.confirmed)

        track.update(detection(1.00, 1.01, 0.69), rospy.Time.from_sec(2.2), 0.20)
        self.assertEqual(fusion._confirm_ready_tracks(), [track])
        self.assertEqual(track.confirmation_mode, "normal_multiframe")

    def test_spatially_inconsistent_pair_does_not_fast_confirm(self):
        track = MineTrack(3, detection(0.0, 0.0, 0.90), rospy.Time.from_sec(1.0))
        track.update(detection(0.20, 0.0, 0.91), rospy.Time.from_sec(1.6), 0.20)
        fusion = make_fusion(track)

        self.assertEqual(fusion._confirm_ready_tracks(), [])
        self.assertFalse(track.confirmed)

    def test_candidate_survives_old_eight_second_window(self):
        track = MineTrack(4, detection(0.0, 0.0, 0.67), rospy.Time.from_sec(10.0))
        fusion = make_fusion(track)

        self.assertFalse(fusion._prune(rospy.Time.from_sec(19.0)))
        self.assertIn(track.id, fusion.tracks)
        self.assertTrue(fusion._prune(rospy.Time.from_sec(41.0)))
        self.assertNotIn(track.id, fusion.tracks)

    def test_confirmed_track_never_times_out(self):
        track = MineTrack(5, detection(0.0, 0.0, 0.90), rospy.Time.from_sec(1.0))
        track.confirmed = True
        fusion = make_fusion(track)

        self.assertFalse(fusion._prune(rospy.Time.from_sec(10000.0)))
        self.assertIn(track.id, fusion.tracks)

    def test_same_xy_high_altitude_detection_cannot_corrupt_ground_track(self):
        track = MineTrack(
            6, detection(2.30, -1.10, 0.90, z=0.04),
            rospy.Time.from_sec(1.0),
        )
        fusion = make_fusion(track)

        assignments, assigned = fusion._associate([
            detection(2.31, -1.09, 0.95, z=3.20)
        ])

        self.assertEqual(assignments, [])
        self.assertEqual(assigned, set())

    def test_confirmed_track_rejects_large_xy_innovation(self):
        track = MineTrack(
            7, detection(2.30, -1.10, 0.90, z=0.04),
            rospy.Time.from_sec(1.0),
        )
        track.confirmed = True
        fusion = make_fusion(track)

        assignments, _assigned = fusion._associate([
            detection(2.50, -1.10, 0.95, z=0.05)
        ])

        self.assertEqual(assignments, [])
        self.assertEqual(
            fusion._duplicate_track_id(
                detection(2.50, -1.10, 0.95, z=0.05)
            ),
            track.id,
        )

    def test_high_altitude_outlier_is_not_duplicate_of_ground_track(self):
        track = MineTrack(
            8, detection(2.30, -1.10, 0.90, z=0.04),
            rospy.Time.from_sec(1.0),
        )
        track.confirmed = True
        fusion = make_fusion(track)

        self.assertIsNone(fusion._duplicate_track_id(
            detection(2.31, -1.09, 0.95, z=3.20)
        ))

    def test_confirmed_identity_removes_nearby_candidate_fragment(self):
        confirmed = MineTrack(
            9, detection(2.30, -1.10, 0.90), rospy.Time.from_sec(1.0)
        )
        confirmed.confirmed = True
        fragment = MineTrack(
            10, detection(2.39, -1.15, 0.82), rospy.Time.from_sec(2.0)
        )
        fusion = make_fusion(confirmed)
        fusion.tracks[fragment.id] = fragment

        self.assertEqual(fusion._suppress_duplicate_tracks(), [fragment.id])
        self.assertEqual(set(fusion.tracks), {confirmed.id})

    def test_confirmed_public_id_beats_lower_numbered_candidate(self):
        fragment = MineTrack(
            9, detection(2.39, -1.15, 0.82), rospy.Time.from_sec(1.0)
        )
        confirmed = MineTrack(
            10, detection(2.30, -1.10, 0.90), rospy.Time.from_sec(2.0)
        )
        confirmed.confirmed = True
        fusion = make_fusion(fragment)
        fusion.tracks[confirmed.id] = confirmed

        self.assertEqual(fusion._suppress_duplicate_tracks(), [fragment.id])
        self.assertEqual(set(fusion.tracks), {confirmed.id})

    def test_near_ugv_detection_is_removed_before_track_processing(self):
        track = MineTrack(
            11, detection(8.0, 8.0, 0.9), rospy.Time.from_sec(1.0)
        )
        fusion = make_fusion(track)
        fusion.ugv_odom_history.append(ugv_sample(20.0, x=2.0, y=-1.0))
        robot_mask = detection(2.8, -1.2, 0.95, z=2.8)

        kept = fusion._exclude_ugv_observations(
            [robot_mask], rospy.Time.from_sec(20.1)
        )

        self.assertEqual(kept, [])
        self.assertEqual(fusion.ugv_rejected_near_total, 1)
        self.assertEqual(fusion.ugv_rejected_high_total, 0)

    def test_elevated_detection_in_wide_ugv_envelope_is_removed(self):
        track = MineTrack(
            12, detection(8.0, 8.0, 0.9), rospy.Time.from_sec(1.0)
        )
        fusion = make_fusion(track)
        fusion.ugv_odom_history.append(ugv_sample(20.0, x=2.0, y=-1.0, z=0.2))
        elevated_arm = detection(4.4, -1.0, 0.95, z=0.71)
        ground_object = detection(4.4, -1.0, 0.90, z=0.04)

        kept = fusion._exclude_ugv_observations(
            [elevated_arm, ground_object], rospy.Time.from_sec(20.1)
        )

        self.assertEqual(kept, [ground_object])
        self.assertEqual(fusion.ugv_rejected_near_total, 0)
        self.assertEqual(fusion.ugv_rejected_high_total, 1)

    def test_far_initial_survey_mine_is_unchanged(self):
        track = MineTrack(
            13, detection(8.0, 8.0, 0.9), rospy.Time.from_sec(1.0)
        )
        fusion = make_fusion(track)
        # The UGV starts near y=-18 while the mine field is near y=0.
        fusion.ugv_odom_history.append(
            ugv_sample(5.0, x=0.0, y=-18.0, z=0.2)
        )
        survey_mine = detection(1.4, 0.9, 0.90, z=0.04)

        kept = fusion._exclude_ugv_observations(
            [survey_mine], rospy.Time.from_sec(5.1)
        )

        self.assertEqual(kept, [survey_mine])
        self.assertEqual(fusion.ugv_rejected_near_total, 0)
        self.assertEqual(fusion.ugv_rejected_high_total, 0)
        self.assertEqual(fusion.ugv_filter_status, "active")

    def test_detection_uses_historical_not_latest_ugv_pose(self):
        track = MineTrack(
            14, detection(8.0, 8.0, 0.9), rospy.Time.from_sec(1.0)
        )
        fusion = make_fusion(track)
        fusion.ugv_odom_history.extend([
            ugv_sample(10.0, x=1.0, y=1.0),
            ugv_sample(14.0, x=9.0, y=9.0),
        ])
        delayed_mask = detection(1.3, 1.1, 0.95, z=2.6)

        kept = fusion._exclude_ugv_observations(
            [delayed_mask], rospy.Time.from_sec(10.2)
        )

        self.assertEqual(kept, [])
        self.assertEqual(fusion.ugv_rejected_near_total, 1)

    def test_missing_or_stale_odometry_fails_open_with_status(self):
        track = MineTrack(
            15, detection(8.0, 8.0, 0.9), rospy.Time.from_sec(1.0)
        )
        fusion = make_fusion(track)
        observation = detection(0.1, 0.1, 0.95, z=2.6)

        self.assertEqual(
            fusion._exclude_ugv_observations(
                [observation], rospy.Time.from_sec(20.0)
            ),
            [observation],
        )
        self.assertEqual(fusion.ugv_filter_status, "no_odometry")

        fusion.ugv_odom_history.append(ugv_sample(10.0))
        self.assertEqual(
            fusion._exclude_ugv_observations(
                [observation], rospy.Time.from_sec(20.0)
            ),
            [observation],
        )
        self.assertEqual(fusion.ugv_filter_status, "stamp_mismatch")
        self.assertEqual(fusion.ugv_filter_bypass_total, 2)

    def test_odometry_frame_is_validated_and_slash_normalized(self):
        track = MineTrack(
            16, detection(8.0, 8.0, 0.9), rospy.Time.from_sec(1.0)
        )
        fusion = make_fusion(track)

        fusion._ugv_odom_cb(odometry(2.0, frame="odom"))
        self.assertEqual(len(fusion.ugv_odom_history), 0)

        fusion._ugv_odom_cb(odometry(2.1, frame="/map", x=3.0))
        self.assertEqual(len(fusion.ugv_odom_history), 1)
        self.assertAlmostEqual(fusion.ugv_odom_history[0].x, 3.0)


if __name__ == "__main__":
    unittest.main()
