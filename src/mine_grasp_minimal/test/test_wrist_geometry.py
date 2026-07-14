#!/usr/bin/env python3
"""Regression checks for gravity-frame top and local-ground geometry."""

import importlib.util
import unittest
from pathlib import Path

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]


def load_localizer_module():
    path = PACKAGE / "scripts" / "wrist_mine_localizer.py"
    spec = importlib.util.spec_from_file_location("wrist_mine_localizer", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


LOCALIZER = load_localizer_module()


class WristGeometryTest(unittest.TestCase):
    def test_zero_colour_diagnostics_preserve_expected_prior_projection(self):
        yellow = np.zeros((12, 16), dtype=bool)
        roi = np.zeros_like(yellow)
        roi[2:11, 3:14] = True
        filtered = np.zeros_like(yellow)

        metadata = LOCALIZER.color_roi_debug_metadata(
            yellow,
            roi,
            filtered,
            expected_pixel=(8.5, 6.0),
            roi_bounds=(3, 2, 14, 11),
            prior_radius=2,
        )

        self.assertEqual(metadata["yellow_pixels_full_raw"], 0)
        self.assertEqual(metadata["yellow_pixels_roi_raw"], 0)
        self.assertEqual(metadata["yellow_pixels_roi_filtered"], 0)
        self.assertEqual(metadata["expected_source_pixel"], [8.5, 6.0])
        self.assertTrue(metadata["expected_source_in_image"])
        self.assertTrue(metadata["expected_source_in_color_roi"])
        self.assertTrue(metadata["expected_source_prior_circle_in_image"])

    def test_colour_diagnostics_separate_raw_roi_and_filtered_counts(self):
        yellow = np.zeros((8, 10), dtype=bool)
        yellow[0, 0] = True
        yellow[3:5, 4:7] = True
        roi = np.zeros_like(yellow)
        roi[2:7, 2:9] = True
        filtered = np.zeros_like(yellow)
        filtered[3:5, 5:7] = True

        metadata = LOCALIZER.color_roi_debug_metadata(
            yellow,
            roi,
            filtered,
            expected_pixel=None,
            roi_bounds=(2, 2, 9, 7),
            prior_radius=3,
        )

        self.assertEqual(metadata["yellow_pixels_full_raw"], 7)
        self.assertEqual(metadata["yellow_pixels_roi_raw"], 6)
        self.assertEqual(metadata["yellow_pixels_roi_filtered"], 4)
        self.assertIsNone(metadata["expected_source_pixel"])
        self.assertIsNone(metadata["expected_source_in_image"])

    def test_robust_top_and_sloped_ground_recover_eighty_five_mm(self):
        rng = np.random.RandomState(17)
        target_xy = np.asarray([0.12, -0.08])

        xx, yy = np.meshgrid(
            np.linspace(-0.20, 0.38, 31),
            np.linspace(-0.31, 0.15, 29),
        )
        ground = 0.37 + 0.18 * (xx - target_xy[0]) - 0.11 * (
            yy - target_xy[1]
        )
        ground_points = np.column_stack((xx.ravel(), yy.ravel(), ground.ravel()))
        ground_points[:, 2] += rng.normal(0.0, 0.001, ground_points.shape[0])
        # Vegetation/collision clutter must not pull the terrain plane upward.
        ground_points[::13, 2] += 0.12

        expected_ground = 0.37
        expected_top = expected_ground + 0.085
        top_points = np.column_stack((
            rng.normal(target_xy[0], 0.012, 160),
            rng.normal(target_xy[1], 0.012, 160),
            rng.normal(expected_top, 0.0012, 160),
        ))
        # A few side/occlusion pixels make a whole-mask median unsafe.
        top_points[:35, 2] -= rng.uniform(0.015, 0.055, 35)

        top, top_count = LOCALIZER.estimate_top_surface(
            top_points, quantile=0.80, band_m=0.006, minimum_samples=12
        )
        self.assertIsNotNone(top)
        self.assertGreaterEqual(top_count, 12)
        ground_z, ground_count = LOCALIZER.fit_local_ground_plane(
            ground_points,
            top[:2],
            minimum_samples=40,
            residual_floor=0.006,
            mad_scale=3.5,
            minimum_xy_span=0.04,
        )
        self.assertIsNotNone(ground_z)
        self.assertGreaterEqual(ground_count, 40)
        self.assertAlmostEqual(float(top[2] - ground_z), 0.085, delta=0.003)
        # The generated grasp target is the 60 mm detonator centre.
        self.assertAlmostEqual(float(top[2] - 0.030), 0.425, delta=0.003)

    def test_ground_plane_is_evaluated_at_target_not_ring_median(self):
        target_xy = np.asarray([0.35, -0.20])
        xx, yy = np.meshgrid(
            np.linspace(-0.45, 0.15, 25),
            np.linspace(-0.25, 0.35, 23),
        )
        zz = 0.5 + 0.65 * xx - 0.30 * yy
        points = np.column_stack((xx.ravel(), yy.ravel(), zz.ravel()))
        expected = 0.5 + 0.65 * target_xy[0] - 0.30 * target_xy[1]
        fitted, count = LOCALIZER.fit_local_ground_plane(
            points, target_xy, 40, 0.006, 3.5, 0.04
        )
        self.assertGreaterEqual(count, 40)
        self.assertAlmostEqual(fitted, expected, places=9)
        self.assertGreater(abs(fitted - float(np.median(zz))), 0.15)

    def test_per_pixel_backprojection_keeps_depth_and_rejects_outlier(self):
        mask = np.zeros((30, 40), dtype=bool)
        mask[8:24, 10:31] = True
        vv, uu = np.indices(mask.shape, dtype=np.float64)
        depth = 1.7 + uu * 0.002 + vv * 0.001
        depth[12, 14] = 8.0
        intrinsics = (120.0, 120.0, 19.5, 14.5)
        cloud, count = LOCALIZER.robust_mask_cloud(
            mask, depth.astype(np.float32), intrinsics,
            0.05, 3.0, 0, 15, 3.5, 0.002,
        )
        self.assertIsNotNone(cloud)
        self.assertGreaterEqual(count, 15)
        self.assertLess(float(np.max(cloud[:, 2])), 3.0)
        recovered_u = cloud[:, 0] * intrinsics[0] / cloud[:, 2] + intrinsics[2]
        self.assertTrue(np.allclose(recovered_u, np.rint(recovered_u), atol=1e-7))


if __name__ == "__main__":
    unittest.main()
