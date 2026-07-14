#!/usr/bin/env python3
"""Regression checks for terrain-relative UAV mine height validation."""

import importlib.util
import unittest
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image


PACKAGE = Path(__file__).resolve().parents[1]


def load_localizer_module():
    path = PACKAGE / "scripts" / "mine_seg_localizer_node.py"
    spec = importlib.util.spec_from_file_location("mine_seg_localizer_node", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MineSegLocalizer = load_localizer_module().MineSegLocalizer


class MineSegLocalGroundTest(unittest.TestCase):
    def setUp(self):
        self.localizer = MineSegLocalizer.__new__(MineSegLocalizer)
        self.localizer.ground_ring_inner_px = 3
        self.localizer.ground_ring_outer_px = 12
        self.localizer.ground_min_samples = 20
        self.localizer.depth_min = 0.2
        self.localizer.depth_max = 20.0
        self.localizer.min_depth_samples = 12
        self.localizer.min_mask_area = 20
        self.localizer.erode_pixels = 0
        self.localizer.depth_mad_scale = 4.0
        self.localizer.depth_mad_floor = 0.03
        self.localizer.ground_plane_mad_scale = 4.0
        self.localizer.ground_plane_residual_floor = 0.025
        self.localizer.ground_plane_min_xy_span = 0.04
        self.localizer.min_height_above_ground = -0.06
        self.localizer.max_height_above_ground = 0.18
        self.localizer.max_rgb_depth_stamp_delta = 0.02

        self.info = CameraInfo()
        self.info.width = 100
        self.info.height = 100
        self.info.K = [100.0, 0.0, 49.5, 0.0, 100.0, 49.5, 0.0, 0.0, 1.0]
        self.mask = np.zeros((100, 100), dtype=bool)
        self.mask[45:55, 45:55] = True

    @staticmethod
    def down_camera_transform(world_height):
        transform = TransformStamped()
        # 180 degrees about camera X makes optical +Z point toward map -Z.
        transform.transform.rotation.x = 1.0
        transform.transform.rotation.w = 0.0
        transform.transform.translation.z = float(world_height)
        return transform

    def test_local_height_is_invariant_to_absolute_hill_elevation(self):
        uu = np.arange(100, dtype=np.float32)[None, :]
        # A shallow depth gradient represents locally sloped terrain.
        depth = np.broadcast_to(2.0 + (uu - 49.5) * 0.001, (100, 100)).copy()

        heights = []
        for camera_world_z in (3.0, 15.0):
            transform = self.down_camera_transform(camera_world_z)
            mine_top = self.localizer._to_map(
                np.asarray([0.0, 0.0, 1.94]), transform
            )
            ground_z, samples = self.localizer._local_ground_z(
                self.mask,
                depth,
                self.info,
                transform,
                target_map_point=mine_top,
            )
            self.assertIsNotNone(ground_z)
            self.assertGreaterEqual(samples, self.localizer.ground_min_samples)
            heights.append(float(mine_top[2] - ground_z))

        self.assertAlmostEqual(heights[0], 0.06, places=2)
        self.assertAlmostEqual(heights[1], 0.06, places=2)
        self.assertAlmostEqual(heights[0], heights[1], places=6)

    def test_mask_pixels_keep_their_own_depth_and_mad_rejects_outlier(self):
        mask = np.zeros((100, 100), dtype=bool)
        mask[42:58, 40:60] = True
        vv, uu = np.indices(mask.shape, dtype=np.float32)
        depth = 2.0 + 0.0015 * (uu - 49.5) + 0.001 * (vv - 49.5)
        depth[45, 45] = 9.0

        measurement = self.localizer._mask_measurement(mask, depth, self.info)
        self.assertIsNotNone(measurement)
        camera_points, depth_m, _area, _u, _v, _used = measurement
        self.assertGreaterEqual(
            camera_points.shape[0], self.localizer.min_depth_samples
        )
        self.assertLess(float(np.max(camera_points[:, 2])), 3.0)
        self.assertAlmostEqual(depth_m, 2.0, places=3)

        # Every reconstructed X must use the very same Z stored in that row.
        recovered_u = camera_points[:, 0] * self.info.K[0] / camera_points[:, 2]
        recovered_u += self.info.K[2]
        self.assertTrue(np.allclose(recovered_u, np.rint(recovered_u), atol=1e-9))

    def test_map_representative_is_median_after_transforming_all_pixels(self):
        mask = np.zeros((100, 100), dtype=bool)
        mask[20:55, 12:30] = True
        mask[42:58, 12:62] = True
        vv, uu = np.indices(mask.shape, dtype=np.float32)
        depth = 2.2 + 0.006 * uu + 0.003 * vv
        measurement = self.localizer._mask_measurement(mask, depth, self.info)
        self.assertIsNotNone(measurement)
        camera_points = measurement[0]

        transform = TransformStamped()
        angle = np.deg2rad(32.0)
        transform.transform.rotation.y = float(np.sin(angle / 2.0))
        transform.transform.rotation.w = float(np.cos(angle / 2.0))
        transformed = self.localizer._points_to_map(camera_points, transform)
        expected = np.median(transformed, axis=0)

        u = measurement[3]
        v = measurement[4]
        z = measurement[1]
        old_synthetic = np.asarray(
            [
                (u - self.info.K[2]) * z / self.info.K[0],
                (v - self.info.K[5]) * z / self.info.K[4],
                z,
            ]
        )
        old_mapped = self.localizer._to_map(old_synthetic, transform)
        self.assertGreater(float(np.linalg.norm(expected - old_mapped)), 0.002)
        self.assertTrue(np.allclose(expected, np.median(transformed, axis=0)))

    def test_steep_edge_slope_uses_plane_at_target_not_ring_median(self):
        info = CameraInfo()
        info.width = 120
        info.height = 100
        info.K = [100.0, 0.0, 59.5, 0.0, 100.0, 49.5, 0.0, 0.0, 1.0]
        mask = np.zeros((100, 120), dtype=bool)
        # A clipped, one-sided ring is the case where median ring z is biased.
        mask[44:56, 2:13] = True

        camera_height = 4.0
        slope_x = 0.70
        slope_y = 0.30
        intercept = 0.65
        vv, uu = np.indices(mask.shape, dtype=np.float64)
        x_norm = (uu - info.K[2]) / info.K[0]
        y_norm = (vv - info.K[5]) / info.K[4]
        depth = (camera_height - intercept) / (
            1.0 + slope_x * x_norm - slope_y * y_norm
        )
        transform = self.down_camera_transform(camera_height)

        target_u = 7.0
        target_v = 49.5
        xn = (target_u - info.K[2]) / info.K[0]
        yn = (target_v - info.K[5]) / info.K[4]
        target_depth = (camera_height - intercept) / (
            1.0 + slope_x * xn - slope_y * yn
        )
        target_ground = self.localizer._to_map(
            np.asarray([xn * target_depth, yn * target_depth, target_depth]),
            transform,
        )
        mine_top = target_ground.copy()
        mine_top[2] += 0.06

        ground_z, samples = self.localizer._local_ground_z(
            mask,
            depth.astype(np.float32),
            info,
            transform,
            target_map_point=mine_top,
        )
        self.assertIsNotNone(ground_z)
        self.assertGreaterEqual(samples, self.localizer.ground_min_samples)
        self.assertAlmostEqual(ground_z, target_ground[2], places=3)
        self.assertAlmostEqual(mine_top[2] - ground_z, 0.06, places=3)

    def test_ground_plane_mad_rejects_nonterrain_points(self):
        xx, yy = np.meshgrid(
            np.linspace(-0.4, 0.4, 21), np.linspace(-0.3, 0.3, 17)
        )
        zz = 0.72 + 0.60 * xx - 0.25 * yy
        points = np.column_stack((xx.ravel(), yy.ravel(), zz.ravel()))
        # Simulate vegetation/building returns mixed into part of the ring.
        points[::11, 2] += 0.45
        ground_z, samples = self.localizer._fit_local_ground_plane(
            points, np.asarray([0.0, 0.0])
        )
        self.assertIsNotNone(ground_z)
        self.assertAlmostEqual(ground_z, 0.72, places=6)
        self.assertLess(samples, points.shape[0])
        self.assertGreaterEqual(samples, self.localizer.ground_min_samples)

    def test_rgb_depth_stamp_gate_returns_depth_stamp(self):
        rgb = Image()
        depth = Image()
        rgb.header.stamp = rospy.Time.from_sec(10.000)
        depth.header.stamp = rospy.Time.from_sec(10.015)
        stamp, error = self.localizer._validated_depth_stamp(rgb, depth)
        self.assertEqual(error, "")
        self.assertEqual(stamp, depth.header.stamp)

        depth.header.stamp = rospy.Time.from_sec(10.021)
        stamp, error = self.localizer._validated_depth_stamp(rgb, depth)
        self.assertIsNone(stamp)
        self.assertIn("exceeds", error)

        rgb.header.stamp = rospy.Time.from_sec(0.0)
        stamp, error = self.localizer._validated_depth_stamp(rgb, depth)
        self.assertIsNone(stamp)
        self.assertIn("zero", error)

    def test_repeated_survey_goal_does_not_defeat_arm_throttle(self):
        localizer = MineSegLocalizer.__new__(MineSegLocalizer)
        localizer.survey_goal_topic = "/uav/survey_goal"
        localizer.last_goal_signatures = {}
        localizer.inference_enabled = False
        localizer.goal_rate_restore_until = 0.0
        localizer.goal_rate_restore_duration = 5.0
        localizer._publish_status = lambda *_args, **_kwargs: None

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = 3.0
        goal.pose.position.z = 4.0
        goal.pose.orientation.w = 1.0
        localizer._goal_cb(goal, "/uav/survey_goal")
        first_deadline = localizer.goal_rate_restore_until
        self.assertTrue(localizer.inference_enabled)

        # A normal survey refresh has a new header stamp but identical geometry.
        localizer.inference_enabled = False
        goal.header.stamp = rospy.Time.from_sec(99.0)
        localizer._goal_cb(goal, "/uav/survey_goal")
        self.assertFalse(localizer.inference_enabled)
        self.assertEqual(localizer.goal_rate_restore_until, first_deadline)

        goal.pose.position.x += 1.0
        localizer._goal_cb(goal, "/uav/survey_goal")
        self.assertTrue(localizer.inference_enabled)
        self.assertGreaterEqual(localizer.goal_rate_restore_until, first_deadline)

    def test_same_manual_goal_is_always_a_new_operator_request(self):
        localizer = MineSegLocalizer.__new__(MineSegLocalizer)
        localizer.survey_goal_topic = "/uav/survey_goal"
        localizer.last_goal_signatures = {}
        localizer.inference_enabled = False
        localizer.goal_rate_restore_until = 0.0
        localizer.goal_rate_restore_duration = 5.0
        localizer._publish_status = lambda *_args, **_kwargs: None
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.orientation.w = 1.0

        localizer._goal_cb(goal, "/uav/manual_goal")
        first_deadline = localizer.goal_rate_restore_until
        localizer.inference_enabled = False
        localizer._goal_cb(goal, "/uav/manual_goal")
        self.assertTrue(localizer.inference_enabled)
        self.assertGreaterEqual(localizer.goal_rate_restore_until, first_deadline)


if __name__ == "__main__":
    unittest.main()
