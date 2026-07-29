#!/usr/bin/env python3
"""Closed-loop lunar pipeline test with A/B metrics.

Two things are measured here. `test_closed_loop` is the A/B regression guard for the
ideal-arc vs. dynamics-primitive comparison (复核复算): planning latency, tracking RMSE
against the published path, and the velocity profile the follower feeds forward from.
`test_metrics_over_trials` drives a fixed tour of goals repeatedly and reports the three
numbers the task book asks for -- 规划成功率, 避障恢复率 and 规划耗时 -- with mean,
standard deviation and extremes, which a single run of a single goal cannot produce.

The goal tour is deterministic rather than random so two runs are comparable. It is
also absolute in the map frame: the rover is never teleported between trials, so each
trial legitimately starts wherever the previous one finished.
"""
import math
import statistics
import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Float32MultiArray, String

# Open terrain well clear of the simulator's rocks (0,-2), (8,-5), (-3,5) and of the
# crater at (5,2). A closed loop, so repeating it returns the rover to where it started
# and later repetitions are not systematically harder than the first.
EASY_GOALS = [(-7.0, -6.0, 0.0),
              (-7.0, -9.0, -math.pi / 2),
              (-10.0, -9.0, math.pi),
              (-10.0, -6.0, math.pi / 2)]

# Deliberately hard, in increasing order of severity.
#   1. Hard against the (0,-2) rock, and 2. inside that rock's occlusion shadow: the goal
#      footprint sits on lethal or unobserved cells, and snapping alone should resolve both
#      without ever entering recovery.
#   3. Dead centre of the largest rock (8,-5): it stands 0.75m over a 0.9m radius, so the
#      nearest pose whose 1.8x1.5m footprint clears it is further than the nominal 1.5m snap
#      radius but well inside the radius Relax doubles that to. This is the goal that makes
#      避障恢复率 measurable -- without a hard-but-reachable case the metric has no
#      denominator and goes unasserted.
#   4. Inside the crater bowl at (5,2): over the slope limit with no solution at all, so the
#      correct outcome is escalating to Abort rather than retrying forever.
HARD_GOALS = [(-1.2, -2.0, 0.0),
              (0.9, -1.6, 0.0),
              (8.0, -5.0, 0.0),
              (3.0, 0.0, math.pi / 4)]

EASY_TIMEOUT = 25.0
HARD_TIMEOUT = 35.0
REACH_TOLERANCE = 0.5


class LunarPipelineTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.last_path = None
        self.last_vel = None
        self.status = None
        self.statuses = set()
        self.first_success_time = None
        self.plan_ms = []
        self.diag = {}
        rospy.Subscriber("/lunar_planner/path", Path, self._on_path, queue_size=1)
        rospy.Subscriber("/lunar_planner/velocity_profile", Float32MultiArray,
                         self._on_vel, queue_size=1)
        rospy.Subscriber("/lunar_planner/status", String, self._on_status, queue_size=1)
        rospy.Subscriber("/lunar_planner/diagnostics", String, self._on_diag, queue_size=10)

    def _on_path(self, msg):
        with self.lock:
            self.last_path = msg

    def _on_vel(self, msg):
        with self.lock:
            self.last_vel = msg

    def _on_status(self, msg):
        with self.lock:
            self.status = msg.data
            self.statuses.add(msg.data)
            if msg.data.startswith("success") and self.first_success_time is None:
                self.first_success_time = rospy.Time.now()

    def _on_diag(self, msg):
        fields = {}
        for token in msg.data.split():
            if "=" in token:
                key, value = token.split("=", 1)
                fields[key] = value
        with self.lock:
            self.diag = fields
            try:
                ms = float(fields.get("plan_ms", "0"))
            except ValueError:
                ms = 0.0
            if ms > 0.0:
                self.plan_ms.append(ms)

    def _counter(self, name):
        with self.lock:
            try:
                return int(self.diag.get(name, "0"))
            except ValueError:
                return 0

    def _cross_track(self, x, y):
        with self.lock:
            path = self.last_path
        if path is None or not path.poses:
            return None
        return min(math.hypot(p.pose.position.x - x, p.pose.position.y - y)
                   for p in path.poses)

    def _reset_trial_state(self):
        with self.lock:
            self.status = None
            self.statuses = set()
            self.first_success_time = None
            self.plan_ms = []

    def _run_trial(self, gx, gy, gyaw, timeout):
        """Send one goal, follow it to completion or failure, return its metrics."""
        self._reset_trial_state()
        events0 = self._counter("recovery_events")
        successes0 = self._counter("recovery_successes")
        aborts0 = self._counter("recovery_aborts")

        initial = rospy.wait_for_message("/localization/odometry/filtered_map", Odometry, timeout=5)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.orientation.z = math.sin(gyaw / 2.0)
        goal.pose.orientation.w = math.cos(gyaw / 2.0)
        pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
        # Published exactly once. Re-sending would reset the planner's stuck-detection
        # state on every tick, so recovery could never trigger and the recovery-rate
        # metric would always read zero.
        rospy.sleep(0.5)
        pub.publish(goal)

        goal_time = rospy.Time.now()
        end = goal_time + rospy.Duration(timeout)
        rate = rospy.Rate(5)
        final = initial
        sq_err = 0.0
        n_err = 0
        reached = False
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            final = rospy.wait_for_message("/localization/odometry/filtered_map", Odometry, timeout=2)
            fx, fy = final.pose.pose.position.x, final.pose.pose.position.y
            ct = self._cross_track(fx, fy)
            if ct is not None:
                sq_err += ct * ct
                n_err += 1
            if math.hypot(fx - gx, fy - gy) < REACH_TOLERANCE:
                reached = True
                break
            with self.lock:
                aborted = self.status == "aborted"
            # The planner has given up and will stay silent until a new goal; waiting out
            # the rest of the timeout would only inflate the test runtime.
            if aborted:
                break
            rate.sleep()

        with self.lock:
            latency = ((self.first_success_time - goal_time).to_sec()
                       if self.first_success_time else float("nan"))
            statuses = set(self.statuses)
            plan_ms = list(self.plan_ms)
        return {
            "goal": (gx, gy),
            "reached": reached,
            "planned": any(s.startswith("success") for s in statuses),
            "statuses": statuses,
            "latency": latency,
            "plan_ms": plan_ms,
            "recovery_events": self._counter("recovery_events") - events0,
            "recovery_successes": self._counter("recovery_successes") - successes0,
            "recovery_aborts": self._counter("recovery_aborts") - aborts0,
            "rmse": math.sqrt(sq_err / n_err) if n_err else float("nan"),
            "moved": math.hypot(final.pose.pose.position.x - initial.pose.pose.position.x,
                                final.pose.pose.position.y - initial.pose.pose.position.y),
            "final_err": math.hypot(final.pose.pose.position.x - gx,
                                    final.pose.pose.position.y - gy),
        }

    def _await_consistent_profile(self, timeout=10.0):
        """Wait for a path and profile snapshot that satisfy the follower's invariant.

        LunarPathFollowerNode::plannedSpeedAt silently ignores the profile unless it holds
        exactly two floats per pose, so this relation -- not merely a non-empty profile --
        is what decides whether the planned speeds reach the wheels. The retry loop exists
        because path and profile are separate publications and a snapshot taken between
        the two is legitimately mismatched.
        """
        end = rospy.Time.now() + rospy.Duration(timeout)
        seen = (0, 0)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            with self.lock:
                n_poses = len(self.last_path.poses) if self.last_path else 0
                n_vel = len(self.last_vel.data) if self.last_vel else 0
            seen = (n_poses, n_vel)
            if n_poses > 0 and n_vel == 2 * n_poses:
                return seen
            rospy.sleep(0.2)
        return seen

    def test_closed_loop(self):
        use_dyn = rospy.get_param("/state_lattice_planner/use_dynamics_primitives", False)
        mode = "dynamics_primitives" if use_dyn else "ideal_arcs"

        cost = rospy.wait_for_message("/terrain/costmap", OccupancyGrid, timeout=20)
        self.assertGreater(sum(1 for v in cost.data if 0 <= v < 100), 1000)
        initial = rospy.wait_for_message("/localization/odometry/filtered_map", Odometry, timeout=5)
        trial = self._run_trial(initial.pose.pose.position.x + 2.0,
                                initial.pose.pose.position.y, 0.0, 35.0)
        n_poses, n_vel = self._await_consistent_profile()

        rospy.loginfo("=== pipeline metrics [mode=%s] ===", mode)
        rospy.loginfo("  moved            = %.3f m", trial["moved"])
        rospy.loginfo("  final goal error = %.3f m", trial["final_err"])
        rospy.loginfo("  tracking RMSE    = %.3f m", trial["rmse"])
        rospy.loginfo("  plan latency     = %.3f s", trial["latency"])
        rospy.loginfo("  path poses / vel = %d / %d", n_poses, n_vel)

        self.assertGreater(trial["moved"], 1.0)
        self.assertLess(trial["final_err"], 0.5)
        # Required in both modes: 3.2 lists the desired linear and angular velocity as a
        # planner output, and the arc mode is the shipping default.
        self.assertGreater(n_poses, 0, "planner published no path")
        self.assertEqual(n_vel, 2 * n_poses,
                         "velocity profile must hold one (v, w) pair per pose")

    def test_metrics_over_trials(self):
        n_trials = int(rospy.get_param("~n_trials", 3))
        rospy.wait_for_message("/terrain/costmap", OccupancyGrid, timeout=20)

        easy, hard = [], []
        for _ in range(n_trials):
            for gx, gy, gyaw in EASY_GOALS:
                easy.append(self._run_trial(gx, gy, gyaw, EASY_TIMEOUT))
        for gx, gy, gyaw in HARD_GOALS:
            hard.append(self._run_trial(gx, gy, gyaw, HARD_TIMEOUT))

        every = easy + hard
        easy_rate = sum(1 for t in easy if t["planned"]) / len(easy)
        overall_rate = sum(1 for t in every if t["planned"]) / len(every)
        events = sum(t["recovery_events"] for t in every)
        successes = sum(t["recovery_successes"] for t in every)
        aborts = sum(t["recovery_aborts"] for t in every)
        # An entry that ends in an abort was a goal with no solution, so it is neither a
        # recovery nor a failed recovery and does not belong in the denominator. Aborting
        # freely cannot game this: every abort also costs a 规划成功率 trial, and both
        # numbers are reported side by side.
        recoverable = events - aborts
        recovery_rate = successes / recoverable if recoverable else float("nan")
        samples = [ms for t in every for ms in t["plan_ms"]]

        rospy.loginfo("=== planner metrics over %d trials ===", len(every))
        rospy.loginfo("  规划成功率 easy   = %.3f (%d/%d)", easy_rate,
                      sum(1 for t in easy if t["planned"]), len(easy))
        rospy.loginfo("  规划成功率 all    = %.3f (%d/%d)", overall_rate,
                      sum(1 for t in every if t["planned"]), len(every))
        rospy.loginfo("  reached goal      = %d/%d", sum(1 for t in every if t["reached"]),
                      len(every))
        rospy.loginfo("  避障恢复率        = %.3f (%d recovered / %d recoverable; "
                      "%d entered, %d aborted as unreachable)",
                      recovery_rate, successes, recoverable, events, aborts)
        if samples:
            rospy.loginfo("  规划耗时 mean/sd  = %.1f / %.1f ms",
                          statistics.mean(samples), statistics.pstdev(samples))
            rospy.loginfo("  规划耗时 min/max  = %.1f / %.1f ms", min(samples), max(samples))
        for t in every:
            rospy.loginfo("    goal (%.1f, %.1f) planned=%s reached=%s statuses=%s",
                          t["goal"][0], t["goal"][1], t["planned"], t["reached"],
                          ",".join(sorted(t["statuses"])))

        # 99% is asserted only on the open-terrain subset. The hard subset exists to
        # exercise snapping and recovery, so holding it to the same bar would turn a
        # deliberately marginal goal into a flaky failure; its rate is reported instead.
        self.assertGreaterEqual(easy_rate, 0.99, "规划成功率 on open terrain")
        self.assertGreaterEqual(overall_rate, 0.90, "规划成功率 including hard goals")
        # Guards the failure mode the split above could otherwise hide: a planner that
        # declares reachable goals unreachable. Open terrain must never abort.
        self.assertFalse([t for t in easy if "aborted" in t["statuses"]],
                         "an open-terrain goal was aborted")
        if recoverable:
            self.assertGreaterEqual(recovery_rate, 0.8,
                                    "避障恢复率: %d recovered of %d recoverable entries"
                                    % (successes, recoverable))
        else:
            rospy.logwarn("避障恢复率 not asserted: no recoverable entries in this run "
                          "(%d entered, all %d aborted as unreachable)", events, aborts)
        self.assertTrue(samples, "no plan_ms samples: /lunar_planner/diagnostics is silent")
        self.assertLess(statistics.mean(samples), 1000.0, "规划耗时 mean exceeds the budget")
        self.assertLess(max(samples), 1200.0, "规划耗时 worst case exceeds the budget")


if __name__ == "__main__":
    rospy.init_node("lunar_pipeline_test")
    rostest.rosrun("groundgrid", "lunar_pipeline_test", LunarPipelineTest)
