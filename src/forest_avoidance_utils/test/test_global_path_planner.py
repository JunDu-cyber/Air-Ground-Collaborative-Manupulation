#!/usr/bin/env python3

import importlib.util
import unittest
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "global_path_planner.py"
SPEC = importlib.util.spec_from_file_location("global_path_planner", SCRIPT)
PLANNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PLANNER)


class GlobalPlannerGeometryTest(unittest.TestCase):
    def setUp(self):
        # Two 5 m-wide, 25 m-long buildings with a 3 m corridor.
        self.raw = np.zeros((80, 80), dtype=bool)
        self.raw[15:65, 20:30] = True
        self.raw[15:65, 36:46] = True
        self.start = (5, 33)
        self.goal = (74, 33)

    def test_one_meter_clearance_keeps_three_meter_corridor(self):
        grid, _clearance = PLANNER.metric_inflation(self.raw, 0.5, 1.0)
        self.assertTrue(PLANNER.line_clear(grid, self.start, self.goal))

    def test_old_clearance_closes_corridor_and_forces_detour(self):
        narrow_grid, narrow_clearance = PLANNER.metric_inflation(self.raw, 0.5, 1.0)
        old_grid, old_clearance = PLANNER.metric_inflation(self.raw, 0.5, 1.5)

        narrow_path = PLANNER.astar(
            narrow_grid, self.start, self.goal, narrow_clearance, 0.5, 1.5, 0.12)
        old_path = PLANNER.astar(
            old_grid, self.start, self.goal, old_clearance, 0.5, 1.5, 0.12)
        self.assertIsNotNone(narrow_path)
        self.assertIsNotNone(old_path)

        narrow_short = PLANNER.shortcut_cells(narrow_path, narrow_grid)
        old_short = PLANNER.shortcut_cells(old_path, old_grid)
        narrow_length = PLANNER.polyline_length(
            [(r * 0.5, c * 0.5) for r, c in narrow_short])
        old_length = PLANNER.polyline_length(
            [(r * 0.5, c * 0.5) for r, c in old_short])

        self.assertEqual(narrow_short, [self.start, self.goal])
        self.assertGreater(old_length, narrow_length + 5.0)

    def test_nearest_free_selection_has_no_scan_direction_bias(self):
        mask = np.zeros((12, 12), dtype=bool)
        mask[3, 3] = True   # distance sqrt(8) from target
        mask[5, 7] = True   # distance 2 from target: the true nearest
        self.assertEqual(PLANNER.nearest_mask_cell(mask, (5, 5), 4.0), (5, 7))

    def test_diagonal_corner_cut_is_rejected(self):
        grid = np.zeros((3, 3), dtype=bool)
        grid[0, 1] = True
        grid[1, 0] = True
        self.assertFalse(PLANNER.line_clear(grid, (0, 0), (1, 1)))
        self.assertIsNone(PLANNER.astar(grid, (0, 0), (1, 1)))

    def test_clear_straight_line_is_euclidean_shortest(self):
        grid = np.zeros((30, 30), dtype=bool)
        clearance = np.full(grid.shape, np.inf, dtype=np.float32)
        path = PLANNER.astar(
            grid, (2, 3), (25, 21), clearance, 0.5, 1.5, 0.12)
        short = PLANNER.shortcut_cells(path, grid)
        self.assertEqual(short, [(2, 3), (25, 21)])

    def test_long_safe_segment_produces_seven_meter_carrot(self):
        planner = PLANNER.GlobalPlanner.__new__(PLANNER.GlobalPlanner)
        planner.lookahead = 7.0
        planner.cur = (0.0, -30.0)
        planner.idx = 1
        planner.path = [(0.0, -30.0), (0.0, 10.0)]
        planner.origin = -50.0
        planner.res = 0.5
        planner.W = 200
        grid = np.zeros((planner.W, planner.W), dtype=bool)
        carrot, exact_index = planner._select_carrot(
            grid, planner.w2c(*planner.cur))
        self.assertIsNone(exact_index)
        self.assertAlmostEqual(carrot[0], 0.0)
        self.assertAlmostEqual(carrot[1], -23.0)


if __name__ == "__main__":
    unittest.main()
