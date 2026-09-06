#!/usr/bin/env python3
"""Goal ownership and atomic terminal-counter regressions, without ROS imports."""
import pathlib
import ast
import math
import json
import sys
import threading
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))
from groundgrid.trial_observation import TrialObservation, fields_of  # noqa: E402
from groundgrid.trial_metrics import mission_completed  # noqa: E402


def planner(stamp=1400, goal=14, seq=1, status="goal_received", events=0, successes=0, aborts=0):
    return fields_of("goal_stamp_ns=%s goal_id=%s snapshot_seq=%s status=%s "
                     "goal_recovery_events=%s goal_recovery_successes=%s goal_recovery_aborts=%s"
                     % (stamp, goal, seq, status, events, successes, aborts))


def follower(goal=14, seq=1, status="tracking"):
    return fields_of("goal_id=%s snapshot_seq=%s status=%s" % (goal, seq, status))


class TrialObservationTest(unittest.TestCase):
    def test_attempt_deduplication_and_ack_race(self):
        state = TrialObservation(1400)
        attempt = dict(source="mission", goal_stamp_ns=1400, goal_id=14,
                       attempt_id=92, total_ms=123.25, ok=True)
        self.assertFalse(state.attempt(attempt))
        self.assertEqual(state.attempts, {})
        state.planner(planner())
        self.assertEqual(len(state.attempts), 1)
        self.assertFalse(state.attempt(attempt))
        state.planner(planner(seq=2))
        self.assertEqual(len(state.attempts), 1)
        self.assertFalse(state.attempt(dict(attempt, source="service", attempt_id=93)))
        self.assertFalse(state.attempt(dict(attempt, goal_id=13, attempt_id=93)))
        self.assertFalse(state.attempt(dict(attempt, total_ms=float("nan"), attempt_id=93)))
        self.assertTrue(state.attempt(dict(attempt, total_ms=0.125, attempt_id=93)))
        self.assertFalse(state.planner_done)

    def test_unarmed_collection_ignores_latched_messages(self):
        state = TrialObservation()
        self.assertFalse(state.planner(planner()))
        self.assertFalse(state.follower(follower()))
        self.assertIsNone(state.status)

    def test_old_abort_cannot_end_new_goal_during_013_second_window(self):
        state = TrialObservation(1400)
        old = planner(stamp=1300, goal=13, seq=100, status="aborted", events=1, aborts=1)
        self.assertFalse(state.planner(old))
        self.assertIsNone(state.status)
        self.assertTrue(state.planner(planner(seq=101)))
        self.assertFalse(state.planner(old))
        self.assertEqual(state.status, "goal_received")
        self.assertNotIn("aborted", state.statuses)
        self.assertEqual(state.counters["recovery_aborts"], 0)

    def test_abort_status_and_counter_are_one_final_snapshot(self):
        state = TrialObservation(1400)
        state.planner(planner(status="recovery_backout", events=1))
        state.planner(planner(seq=2, status="aborted", events=1, aborts=1))
        self.assertEqual(state.status, "aborted")
        self.assertEqual(state.counters["recovery_aborts"], 1)
        self.assertEqual(state.diagnostics["goal_recovery_aborts"], "1")
        self.assertFalse(state.planner(planner(status="recovery_backout", events=1)))
        self.assertEqual(state.status, "aborted")

    def test_no_cross_goal_counter_subtraction(self):
        state = TrialObservation(1500)
        self.assertFalse(state.planner(planner(status="aborted", events=1, aborts=1)))
        self.assertTrue(state.planner(planner(stamp=1500, goal=15, seq=200,
                                             status="aborted", events=2, successes=1, aborts=1)))
        self.assertEqual(state.counters, {"recovery_events": 2,
                                         "recovery_successes": 1, "recovery_aborts": 1})

    def test_follower_before_ack_is_replayed_only_for_acknowledged_goal(self):
        state = TrialObservation(1400)
        state.follower(follower(goal=13, status="goal_reached"))
        state.follower(follower(seq=2))
        state.follower(follower(seq=3, status="goal_reached"))
        self.assertFalse(state.follower_done)
        state.planner(planner())
        self.assertTrue(state.follower_done)
        self.assertFalse(state.planner_done)
        state.planner(planner(seq=2, status="goal_reached"))
        self.assertTrue(state.planner_done)

    def test_old_follower_completion_and_stale_are_ignored(self):
        state = TrialObservation(1400)
        state.planner(planner())
        state.follower(follower())
        self.assertFalse(state.follower(follower(goal=13, seq=50, status="goal_reached")))
        self.assertFalse(state.follower(follower(goal=13, seq=51, status="stale_terrain")))
        self.assertFalse(state.follower_done)
        self.assertFalse(state.follower_stale)
        state.follower(follower(seq=2, status="stale_terrain"))
        self.assertTrue(state.follower_stale)

    def test_new_tracking_does_not_inherit_recovery_manoeuvre_completion(self):
        state = TrialObservation(1400)
        state.planner(planner())
        state.follower(follower())
        state.follower(follower(seq=2, status="goal_reached"))
        self.assertTrue(state.follower_done)
        state.follower(follower(seq=3))
        self.assertFalse(state.follower_done)
        self.assertFalse(state.follower(follower(seq=2, status="goal_reached")))
        self.assertFalse(state.follower_done)

    def test_missing_or_regressing_counters_are_not_fabricated(self):
        state = TrialObservation(1400)
        missing = planner()
        del missing["goal_recovery_aborts"]
        self.assertFalse(state.planner(missing))
        self.assertFalse(state.planner(planner(events=-1)))
        self.assertTrue(state.planner(planner(events=1)))
        self.assertFalse(state.planner(planner(seq=2, events=0)))
        self.assertEqual(state.counters["recovery_events"], 1)

    def test_stop_acknowledgement_is_neither_arrival_nor_stale(self):
        state = TrialObservation(1400)
        state.planner(planner())
        state.follower(follower())
        state.follower(follower(seq=2, status="goal_reached"))
        message = follower(seq=3, status="empty_trajectory")
        message["trajectory_stamp_ns"] = "1788428079729038841"
        self.assertTrue(state.follower(message))
        self.assertFalse(state.follower_done)
        self.assertFalse(state.planner_done)
        self.assertFalse(state.follower_stale)
        self.assertEqual(state.status, "goal_received")

    def test_another_id_cannot_rebind_an_acknowledged_stamp(self):
        state = TrialObservation(1400)
        state.planner(planner())
        self.assertTrue(state.owns(14))
        self.assertFalse(state.owns(13))
        self.assertFalse(state.planner(planner(goal=15, seq=2, status="aborted")))
        self.assertEqual(state.goal_id, 14)

    def test_adjacent_large_nanosecond_stamps_are_not_rounded_together(self):
        stamp = 1788405788378267800
        state = TrialObservation(stamp)
        self.assertFalse(state.planner(planner(stamp=stamp-1)))
        self.assertTrue(state.planner(planner(stamp=stamp)))


class PipelineCallbackTest(unittest.TestCase):
    """Run the actual test adapter methods with message-shaped objects, without ROS.

    Only extract the class definition, avoiding ROS imports and the rostest entry point.
    These tests cover the wiring to the reducer as well as the reducer's own unit tests.
    """
    def setUp(self):
        source = pathlib.Path(__file__).with_name("test_lunar_pipeline.py")
        node = next(n for n in ast.parse(source.read_text(encoding="utf-8")).body
                    if isinstance(n, ast.ClassDef) and n.name == "LunarPipelineTest")
        namespace = {"unittest": unittest, "math": math, "json": json,
                     "TrialObservation": TrialObservation, "fields_of": fields_of,
                     "mission_completed": mission_completed,
                     "STRICT_SCENARIOS": ("mixed", "flat"), "UNREACHABLE_GOALS": {(3.0, 0.0)},
                     "rospy": NS(Time=NS(now=lambda: 100.0))}
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), "exec"), namespace)
        self.adapter = namespace["LunarPipelineTest"]()
        self.adapter.lock = threading.Lock()
        self.adapter.pending_trajectories = {}
        self.adapter.cpu_pct, self.adapter.rss_mb = [], []
        self.adapter._reset_trial_state()
        self.adapter.observation = TrialObservation(1400)

    def diagnostic(self, fields, follower_message=False):
        message = NS(data=" ".join("%s=%s" % item for item in fields.items()))
        method = self.adapter._on_follower_status if follower_message else self.adapter._on_diag
        method(message)

    @staticmethod
    def trajectory(goal=14, empty=False, invalid=False):
        pose = NS(pose=NS(position=NS(x=0.0, y=0.0, z=0.0),
                          orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))
        twist = NS(linear=NS(x=0.5), angular=NS(z=0.1))
        return NS(path=NS(header=NS(seq=goal), poses=[] if empty else [pose]),
                  twists=[] if empty or invalid else [twist])

    def test_atomic_path_before_ack_is_not_lost_or_assigned_to_old_goal(self):
        self.adapter._on_trajectory(self.trajectory(goal=13))
        self.adapter._on_trajectory(self.trajectory())
        self.assertFalse(self.adapter.path_seen)
        self.diagnostic(planner())
        self.assertTrue(self.adapter.path_seen)
        self.assertTrue(self.adapter.atomic_profile_ok)
        self.assertEqual(self.adapter.last_path.header.seq, 14)
        self.assertEqual(self.adapter.last_profile_size, 2)

    def test_actual_attempt_callback_not_republished_plan_ms(self):
        record = dict(source="mission", goal_stamp_ns=1400, goal_id=14,
                      attempt_id=92, total_ms=12.5)
        self.adapter._on_attempt(NS(data=json.dumps(record)))
        self.diagnostic(planner())
        self.adapter._on_attempt(NS(data=json.dumps(record)))
        self.diagnostic(dict(planner(seq=2), plan_ms="12.5"))
        self.assertEqual(len(self.adapter.observation.attempts), 1)
        self.assertEqual(self.adapter.plan_ms, [])

    def test_late_old_stop_and_abort_do_not_clear_current_goal(self):
        self.diagnostic(planner())
        self.adapter._on_trajectory(self.trajectory())
        self.adapter._on_trajectory(self.trajectory(goal=13, empty=True))
        self.diagnostic(planner(stamp=1300, goal=13, status="aborted"))
        self.assertEqual(len(self.adapter.last_path.poses), 1)
        self.assertEqual(self.adapter.status, "goal_received")

    def test_terminal_flags_come_only_from_matching_snapshots(self):
        self.diagnostic(planner())
        self.diagnostic(follower(goal=13, status="goal_reached"), True)
        self.assertFalse(self.adapter.follower_goal_reached)
        self.diagnostic(follower(), True)
        self.diagnostic(follower(seq=2, status="goal_reached"), True)
        self.diagnostic(planner(seq=2, status="goal_reached"))
        self.assertTrue(self.adapter.planner_goal_reached)
        self.assertTrue(self.adapter.follower_goal_reached)

    def test_invalid_current_atomic_profile_still_fails_conformance(self):
        self.diagnostic(planner())
        self.adapter._on_trajectory(self.trajectory(invalid=True))
        self.assertTrue(self.adapter.path_seen)
        self.assertTrue(self.adapter.atomic_profile_invalid)

    def test_corrected_recovery_rate_cannot_hide_uncompleted_hard_goal(self):
        self.adapter.scenario = "mixed"
        tour = [{"goal": (-7.0, -6.0), "statuses": {"goal_reached"},
                 "end_reason": "completed", "planner_goal_reached": True,
                 "follower_goal_reached": True}]
        hard = {"goal": (-1.2, -2.0), "statuses": {"success_snapped", "aborted"},
                "end_reason": "aborted", "planner_goal_reached": False,
                "follower_goal_reached": False}
        report = {"counts": {"unacknowledged_goals": 0, "collisions": 0},
                  "collided": [], "metrics": {},
                  "rates": {"plan_success_tour": 1.0, "plan_success_all": 1.0,
                            "reach_tour": 1.0, "completion_tour": 1.0,
                            "near_obstacle_recovery": 1.0}}
        with self.assertRaisesRegex(AssertionError, "solvable hard goals must complete"):
            self.adapter._assert(report, tour, tour + [hard])


if __name__ == "__main__":
    unittest.main()
