#!/usr/bin/env python3
"""Completion-accounting regressions without a ROS master or ROS Python imports."""

import itertools
import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))

from groundgrid.trial_metrics import completion_summary, mission_completed  # noqa: E402


class TrialMetricsTest(unittest.TestCase):
    def test_both_completion_signals_are_required(self):
        for planner, follower in itertools.product((False, True), repeat=2):
            with self.subTest(planner=planner, follower=follower):
                trial = {"reached": True, "planned": True, "statuses": ["success"],
                         "planner_goal_reached": planner, "follower_goal_reached": follower}
                self.assertEqual(mission_completed(trial), planner and follower)

    def test_missing_or_nonboolean_signals_are_not_success(self):
        self.assertFalse(mission_completed({"reached": True, "planned": True}))
        self.assertFalse(mission_completed({"planner_goal_reached": True}))
        self.assertFalse(mission_completed({"follower_goal_reached": True}))
        self.assertFalse(mission_completed({"planner_goal_reached": "False",
                                            "follower_goal_reached": "True"}))

    def test_aborted_or_timed_out_trial_is_not_completed(self):
        trial = {"planner_goal_reached": True, "follower_goal_reached": True}
        for reason in ("timeout", "aborted", "shutdown"):
            self.assertFalse(mission_completed(dict(trial, end_reason=reason)))
        self.assertTrue(mission_completed(dict(trial, end_reason="completed")))

    def test_snapped_endpoint_completion_is_distinct_from_requested_reach(self):
        trial = {"reached": False, "planner_goal_reached": True,
                 "follower_goal_reached": True, "end_reason": "completed",
                 "statuses": ["success_snapped", "goal_reached"]}
        self.assertTrue(mission_completed(trial))
        self.assertFalse(trial["reached"])

    def test_reported_878cbdf_mixed_n3_cannot_pass_completion_bar(self):
        # Reported pattern: all 12 tour legs entered the radius, but leg 10 never got
        # either completion signal. Four hard goals followed; none completed.
        tour = [{"reached": True, "planner_goal_reached": i != 9,
                 "follower_goal_reached": i != 9} for i in range(12)]
        every = tour + [{"reached": False, "planner_goal_reached": False,
                         "follower_goal_reached": False} for _ in range(4)]
        summary = completion_summary(every, tour)
        self.assertEqual(sum(t["reached"] for t in tour) / len(tour), 1.0)
        self.assertEqual(summary["counts"]["missions_completed"], 11)
        self.assertEqual(summary["counts"]["tour_missions_completed"], 11)
        self.assertAlmostEqual(summary["rates"]["completion_tour"], 11 / 12)
        self.assertAlmostEqual(summary["rates"]["completion_rate"], 11 / 16)
        self.assertLess(summary["rates"]["completion_tour"], 0.99)

    def test_all_completed_tour_passes(self):
        tour = [{"planner_goal_reached": True, "follower_goal_reached": True,
                 "end_reason": "completed"} for _ in range(12)]
        summary = completion_summary(tour, tour)
        self.assertEqual(summary["rates"]["completion_rate"], 1.0)
        self.assertEqual(summary["rates"]["completion_tour"], 1.0)

    def test_empty_sample_sets_are_not_fabricated_success(self):
        summary = completion_summary([], [])
        self.assertEqual(summary["counts"]["missions_completed"], 0)
        self.assertTrue(math.isnan(summary["rates"]["completion_rate"]))
        self.assertTrue(math.isnan(summary["rates"]["completion_tour"]))


if __name__ == "__main__":
    unittest.main()
