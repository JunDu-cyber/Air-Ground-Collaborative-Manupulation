#!/usr/bin/env python3
"""Regression tests for survey completion and generic reverse rescans."""

import importlib.util
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace


PACKAGE = Path(__file__).resolve().parents[1]


def load_survey_module():
    path = PACKAGE / "scripts" / "mine_survey_waypoints.py"
    spec = importlib.util.spec_from_file_location("mine_survey_waypoints", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SURVEY_MODULE = load_survey_module()
ordered_route = SURVEY_MODULE.ordered_route
survey_completion_ready = SURVEY_MODULE.survey_completion_ready
MineSurveyWaypoints = SURVEY_MODULE.MineSurveyWaypoints


class MineSurveyWaypointsLogicTest(unittest.TestCase):
    def test_unreceived_map_never_completes(self):
        self.assertFalse(survey_completion_ready(False, set(), set(), 0))

    def test_configured_confirmed_count_with_no_candidates_completes(self):
        self.assertTrue(
            survey_completion_ready(True, {1, 2, 3, 4, 5}, set(), 5)
        )

    def test_four_of_five_never_completes(self):
        self.assertFalse(
            survey_completion_ready(True, {1, 2, 3, 4}, set(), 5)
        )

    def test_candidate_blocks_complete_even_after_minimum_is_met(self):
        self.assertFalse(
            survey_completion_ready(True, {1, 2, 3, 4, 5}, {6}, 5)
        )

    def test_map_callback_tracks_completion_even_when_hover_is_disabled(self):
        survey = MineSurveyWaypoints.__new__(MineSurveyWaypoints)
        survey.lock = threading.Lock()
        survey.map_received = False
        survey.map_revision = 0
        survey.map_last_wall = None
        survey.confirmed_ids = set()
        survey.unconfirmed_ids = set()
        survey.hold_candidate_ids = set()
        survey.hold_pose = None
        survey.hold_until_wall = 0.0
        survey.candidate_hold_duration = 0.0
        survey.minimum_confirmed_mines = 2
        message = SimpleNamespace(
            revision=7,
            mines=[
                SimpleNamespace(id=1, confirmed=True),
                SimpleNamespace(id=2, confirmed=True),
                SimpleNamespace(id=3, confirmed=False),
            ],
        )

        survey._mine_map_cb(message)

        ready, snapshot = survey._completion_snapshot()
        self.assertFalse(ready)
        self.assertTrue(snapshot["received"])
        self.assertEqual(snapshot["revision"], 7)
        self.assertEqual(snapshot["confirmed_ids"], {1, 2})
        self.assertEqual(snapshot["candidate_ids"], {3})

    def test_rescan_route_alternates_without_mine_coordinates(self):
        route = [(2.5, -0.5), (5.5, 0.4), (8.5, -0.4)]
        self.assertEqual(ordered_route(route, 0), route)
        self.assertEqual(ordered_route(route, 1), list(reversed(route)))
        self.assertEqual(ordered_route(route, 2), route)
        self.assertEqual(route[0], (2.5, -0.5))


if __name__ == "__main__":
    unittest.main()
