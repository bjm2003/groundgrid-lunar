#!/usr/bin/env python3
"""Closed-loop lunar pipeline test with A/B metrics.

Two things are measured here. `test_closed_loop` is the A/B regression guard for the
ideal-arc vs. dynamics-primitive comparison (复核复算): planning latency, tracking RMSE
against the published path, and the velocity profile the follower feeds forward from.
`test_metrics_over_trials` drives a fixed tour of goals repeatedly over one terrain class
and reports the full metric set the task book asks for -- 规划成功率, 避障成功率,
近障恢复率, 规划耗时, 路径长度偏差, 轨迹跟踪误差, 安全距离, 资源占用超标率,
异常发生率, 输出达标率 -- each with mean, standard deviation and extremes, which a
single run of a single goal cannot produce.

The terrain class comes from `~scenario` and must match the one `lunar_surface_sim` was
launched with; `scripts/run_planner_experiments.py` sweeps all five. The goal tour is
deterministic rather than random so two runs are comparable, and it is a closed loop in
the map frame -- the rover is never teleported between trials, so the Nth repetition is
not systematically harder than the first.

安全距离 and collisions are measured against the simulator's ground-truth hazard list on
`/lunar_sim/obstacles`, not against the published costmap, which keeps "did the planner
avoid the obstacle" separate from "did perception see it".
"""
import json
import math
import os
import statistics
import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Float32MultiArray, String
from groundgrid.msg import LunarTrajectory

# The body, from state_lattice_planner/footprint_{length,width}. Clearance is measured
# against this rectangle rather than its circumscribed circle: the circle sits 0.42 m
# outside the body broadside, which is more margin than the planner is obliged to leave,
# so it reports collisions for poses the planner correctly considers safe.
FOOTPRINT_LENGTH = 1.8
FOOTPRINT_WIDTH = 1.5

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
#      radius but well inside the radius Relax doubles that to.
#   4. Inside the crater bowl at (5,2): over the slope limit with no solution at all, so the
#      correct outcome is escalating to Abort rather than retrying forever.
# Goal 2 is the one that actually feeds the 近障恢复率 denominator (measured: it is the
# only goal that walks the full no_path -> recovery_relax -> success chain), so it must
# not be dropped when this list is edited.
HARD_GOALS = [(-1.2, -2.0, 0.0),
              (0.9, -1.6, 0.0),
              (8.0, -5.0, 0.0),
              (3.0, 0.0, math.pi / 4)]

# Goals whose correct outcome is failure. (3.0, 0.0) sits 2.83 m from the crater centre,
# inside the 3.2 m bowl, where the wall runs at 40 deg against a 20 deg limit -- there is
# no solution and Abort is the right answer. Counting that as a 规划成功率 failure caps the
# metric at 15/16 = 0.9375 against a 0.90 bar, so the designed outcome alone consumed all
# the headroom and one ordinary flake failed the run. They are excluded from the rate and
# asserted on directly instead, which is the stricter test: the run now fails if the
# planner *succeeds* here, which the old form could not detect at all.
UNREACHABLE_GOALS = {(3.0, 0.0)}

# Corridor intersections of the 4.0 m boulder grid: the columns sit near x = -6,-2,2,6 and
# the rows near y = -10,-6,-2, so x = -4,0,4 and y = -8,-4 are the gaps. Every waypoint
# below clears the nearest boulder by ~0.5 m after the body diagonal, and the longest leg
# is 8.9 m, which keeps a 10-repetition sweep inside the rostest time limit.
DENSE_GOALS = [(-4.0, -8.0, 0.0),
               (4.0, -4.0, math.pi / 4),
               (-4.0, -4.0, math.pi),
               (-11.0, -8.0, math.pi)]

# Inside the largest passable component of the slope field (x -5..5, y -12..1); the rest
# of the window is over the 20 deg limit, so a tour that left the component would measure
# the terrain rather than the planner.
SLOPE_GOALS = [(4.5, -8.5, 0.0),
               (-1.0, -12.0, math.pi),
               (-5.0, -3.5, math.pi / 2),
               (0.0, -5.5, 0.0)]

# The pit cluster's internal gaps are 0.1-1.8 m against a 1.5 m body, so it is effectively
# one blob and the tour has to round its southern edge rather than thread it. Out and back
# along the same corridor keeps every leg under 7.2 m.
NEGATIVE_GOALS = [(-8.0, -11.0, 0.0),
                  (-1.0, -12.0, 0.0),
                  (-8.0, -11.0, math.pi),
                  (-10.0, -6.0, math.pi / 2)]

# tour: repeated n_trials times. hard: run once, after the tour.
SCENARIOS = {
    "mixed": {"tour": EASY_GOALS, "timeout": 25.0,
              "hard": HARD_GOALS, "hard_timeout": 35.0},
    "flat": {"tour": EASY_GOALS, "timeout": 25.0},
    "dense": {"tour": DENSE_GOALS, "timeout": 50.0},
    "slope": {"tour": SLOPE_GOALS, "timeout": 50.0},
    "negative": {"tour": NEGATIVE_GOALS, "timeout": 45.0},
}

# Where the tour is open terrain by construction and the success rate is therefore a real
# bar rather than a measurement. The other three classes are deliberately marginal: holding
# them to 99% would turn a designed edge case into a flaky failure, so their rates are
# reported and only collisions and the resource budget are asserted.
STRICT_SCENARIOS = ("mixed", "flat")

REACH_TOLERANCE = 0.5
NOMINAL_STATUSES = ("success", "success_snapped", "goal_reached")

# A published path counts as "the route for this trial" only if it starts where the trial
# started and ends at the trial's goal. Without this, 路径长度偏差 measured whatever path
# happened to arrive first after the goal was sent -- usually a straggling replan of the
# previous goal or a recovery manoeuvre, both far shorter than the leg they were credited
# to, which is how the ratio came out at -0.31 (a route shorter than the straight line it
# is compared against is not a detour, it is the wrong path).
# The start tolerance is one body length: the rover keeps moving while the goal is in
# flight. The end tolerance covers a snapped goal (max_snap_distance 1.5 m, doubled by the
# Relax rung) plus the follower's position tolerance.
ROUTE_START_TOLERANCE = 2.0
ROUTE_END_TOLERANCE = 3.5


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stats(values):
    """mean / sd / min / max over the finite samples, as the task book asks for."""
    vals = [v for v in values if isinstance(v, float) and v == v and abs(v) != float("inf")]
    if not vals:
        return {"n": 0, "mean": None, "sd": None, "min": None, "max": None}
    return {"n": len(vals), "mean": statistics.mean(vals), "sd": statistics.pstdev(vals),
            "min": min(vals), "max": max(vals)}


def _describe_collision(c):
    """One line per collision: which goal, where the body was, and what it hit."""
    at = "(%.2f, %.2f)" % tuple(c["at"]) if c["at"] else "unknown pose"
    if c["obstacle"]:
        ox, oy, radius, height = c["obstacle"]
        what = "%s r=%.2f h=%+.2f at (%.2f, %.2f)" % (
            "crater" if height < 0 else "rock", radius, height, ox, oy)
    else:
        what = "unknown hazard"
    return "    goal (%.1f, %.1f): clearance %.2f m at %s vs %s [%s]" % (
        c["goal"][0], c["goal"][1], c["clearance_min_m"], at, what,
        ",".join(c["statuses"]))


class LunarPipelineTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.scenario = rospy.get_param("~scenario", "mixed")
        if self.scenario not in SCENARIOS:
            rospy.logwarn("unknown scenario '%s', falling back to 'mixed'", self.scenario)
            self.scenario = "mixed"
        self.cpu_budget = float(rospy.get_param("~cpu_budget_pct", 40.0))
        self.last_path = None
        self.last_vel = None
        self.status = None
        self.statuses = set()
        self.first_success_time = None
        self.plan_ms = []
        self.plan_path_len = None
        self.plan_chord = None
        self.route_from = None
        self.route_to = None
        self.path_seen = False
        self.profile_ok = False
        self.atomic_profile_ok = False
        self.atomic_profile_invalid = False
        self.angular_profile_seen = False
        self.follower_tracking_seen = False
        self.follower_goal_reached = False
        self.planner_goal_reached = False
        self.follower_stale = False
        self.cpu_pct = []
        self.rss_mb = []
        self.diag = {}
        self.obstacles = []
        rospy.Subscriber("/lunar_planner/path", Path, self._on_path, queue_size=1)
        rospy.Subscriber("/lunar_planner/velocity_profile", Float32MultiArray,
                         self._on_vel, queue_size=1)
        rospy.Subscriber("/lunar_planner/trajectory", LunarTrajectory,
                         self._on_trajectory, queue_size=1)
        rospy.Subscriber("/lunar_planner/status", String, self._on_status, queue_size=1)
        rospy.Subscriber("/lunar_path_follower/status", String,
                         self._on_follower_status, queue_size=10)
        rospy.Subscriber("/lunar_planner/diagnostics", String, self._on_diag, queue_size=10)
        try:
            self._on_obstacles(rospy.wait_for_message("/lunar_sim/obstacles",
                                                      Float32MultiArray, timeout=10))
        except rospy.ROSException:
            # Clearance and collision then go unmeasured, which is worth a loud warning
            # but not worth failing the latency and success-rate metrics over.
            rospy.logwarn("no /lunar_sim/obstacles: 安全距离 and 避障成功率 unavailable")

    def _on_obstacles(self, msg):
        data = list(msg.data)
        with self.lock:
            self.obstacles = [tuple(data[i:i + 4]) for i in range(0, len(data) - 3, 4)]

    def _on_path(self, msg):
        with self.lock:
            self.last_path = msg
            if len(msg.poses) > 0:
                self.path_seen = True
            # The first plan after a goal is the whole route; later ones are the shrinking
            # remainder, so only the first is comparable against the straight-line distance.
            # It has to be a plan *for this trial* -- see ROUTE_START_TOLERANCE.
            if self.plan_path_len is None and len(msg.poses) > 1 and self.route_to:
                first = msg.poses[0].pose.position
                last = msg.poses[-1].pose.position
                gx, gy = self.route_to
                x0, y0 = self.route_from
                if (math.hypot(first.x - x0, first.y - y0) <= ROUTE_START_TOLERANCE and
                        math.hypot(last.x - gx, last.y - gy) <= ROUTE_END_TOLERANCE and
                        math.hypot(last.x - gx, last.y - gy) <
                        math.hypot(first.x - gx, first.y - gy)):
                    self.plan_path_len = sum(
                        math.hypot(b.pose.position.x - a.pose.position.x,
                                   b.pose.position.y - a.pose.position.y)
                        for a, b in zip(msg.poses, msg.poses[1:]))
                    # The chord of this path, not the trial's start-to-goal line. Those
                    # differ by the metre or so the rover covers while the goal is in
                    # flight and by however far the goal had to be snapped, and both
                    # differences are subtracted from the route without being subtracted
                    # from the line, which is what drove the ratio negative. Against its
                    # own chord the quantity is a detour by construction: >= 0, zero for a
                    # straight run, and positive only where the planner routed around
                    # something.
                    self.plan_chord = math.hypot(last.x - first.x, last.y - first.y)

    def _on_vel(self, msg):
        with self.lock:
            self.last_vel = msg
            # Checked here rather than by polling both topics: the profile is published
            # after the path it belongs to, so at this instant last_path is its match.
            n_poses = len(self.last_path.poses) if self.last_path else 0
            if n_poses > 0 and len(msg.data) == 2 * n_poses:
                self.profile_ok = True

    def _on_trajectory(self, msg):
        with self.lock:
            n_poses = len(msg.path.poses)
            valid = len(msg.twists) == n_poses
            if valid:
                valid = all(math.isfinite(pose.pose.position.x) and
                            math.isfinite(pose.pose.position.y) and
                            math.isfinite(pose.pose.position.z) and
                            math.isfinite(pose.pose.orientation.x) and
                            math.isfinite(pose.pose.orientation.y) and
                            math.isfinite(pose.pose.orientation.z) and
                            math.isfinite(pose.pose.orientation.w) and
                            math.isfinite(twist.linear.x) and
                            math.isfinite(twist.angular.z) and
                            abs(twist.linear.x) <= 1.39 + 1e-4 and
                            abs(twist.angular.z) <= 0.8 + 1e-4
                            for pose, twist in zip(msg.path.poses, msg.twists))
            if not valid:
                self.atomic_profile_invalid = True
            elif n_poses > 0:
                self.atomic_profile_ok = True
                if any(abs(twist.angular.z) > 1e-3 for twist in msg.twists):
                    self.angular_profile_seen = True

    def _on_follower_status(self, msg):
        with self.lock:
            if msg.data == "tracking":
                self.follower_tracking_seen = True
            elif msg.data == "goal_reached" and self.follower_tracking_seen:
                self.follower_goal_reached = True
            elif (self.follower_tracking_seen and
                  msg.data in ("stale_trajectory", "stale_terrain")):
                self.follower_stale = True

    def _on_status(self, msg):
        with self.lock:
            self.status = msg.data
            self.statuses.add(msg.data)
            if msg.data == "goal_reached" and self.path_seen:
                self.planner_goal_reached = True
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
            # The planner reports -1 when /proc is unreadable and on its first sample,
            # where there is no interval to divide by. Dropping those is the point: a
            # fabricated zero would silently satisfy the budget assertion.
            for key, sink in (("cpu_pct", self.cpu_pct), ("rss_mb", self.rss_mb)):
                try:
                    value = float(fields.get(key, "-1"))
                except ValueError:
                    continue
                if value >= 0.0:
                    sink.append(value)

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

    def _clearance(self, x, y, yaw):
        """Gap to the nearest ground-truth hazard, and which hazard it was.

        Negative means the hazard circle overlaps the body, which is the collision
        criterion. A hazard whose centre lies under the body reads as -radius, so the
        depth of the negative number distinguishes "clipped the skirt" from "parked on
        top of it". Returns None when no ground truth has arrived.
        """
        with self.lock:
            obstacles = self.obstacles
        if not obstacles:
            return None
        cyaw, syaw = math.cos(yaw), math.sin(yaw)
        worst = None
        for ox, oy, radius, height in obstacles:
            dx, dy = ox - x, oy - y
            # Hazard centre in the body frame, then the distance from it to the rectangle.
            px = cyaw * dx + syaw * dy
            py = -syaw * dx + cyaw * dy
            ex = max(abs(px) - FOOTPRINT_LENGTH / 2.0, 0.0)
            ey = max(abs(py) - FOOTPRINT_WIDTH / 2.0, 0.0)
            gap = math.hypot(ex, ey) - radius
            if worst is None or gap < worst[0]:
                worst = (gap, (ox, oy, radius, height))
        return worst

    def _reset_trial_state(self):
        with self.lock:
            self.status = None
            self.statuses = set()
            self.first_success_time = None
            self.plan_ms = []
            self.plan_path_len = None
            self.plan_chord = None
            self.route_from = None
            self.route_to = None
            self.path_seen = False
            self.profile_ok = False
            self.atomic_profile_ok = False
            self.atomic_profile_invalid = False
            self.angular_profile_seen = False
            self.follower_tracking_seen = False
            self.follower_goal_reached = False
            self.planner_goal_reached = False
            self.follower_stale = False

    def _run_trial(self, gx, gy, gyaw, timeout):
        """Send one goal, follow it to completion or failure, return its metrics."""
        self._reset_trial_state()
        events0 = self._counter("recovery_events")
        successes0 = self._counter("recovery_successes")
        aborts0 = self._counter("recovery_aborts")

        initial = rospy.wait_for_message("/localization/odometry/filtered_map", Odometry, timeout=5)
        x0, y0 = initial.pose.pose.position.x, initial.pose.pose.position.y
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.orientation.z = math.sin(gyaw / 2.0)
        goal.pose.orientation.w = math.cos(gyaw / 2.0)
        with self.lock:
            self.route_from = (x0, y0)
            self.route_to = (gx, gy)
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
        driven = 0.0
        prev = (x0, y0)
        clearances = []
        worst = None
        reached = False
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            final = rospy.wait_for_message("/localization/odometry/filtered_map", Odometry, timeout=2)
            fx, fy = final.pose.pose.position.x, final.pose.pose.position.y
            driven += math.hypot(fx - prev[0], fy - prev[1])
            prev = (fx, fy)
            ct = self._cross_track(fx, fy)
            if ct is not None:
                sq_err += ct * ct
                n_err += 1
            near = self._clearance(fx, fy, _yaw_of(final.pose.pose.orientation))
            if near is not None:
                clearances.append(near[0])
                if worst is None or near[0] < worst[0]:
                    worst = (near[0], near[1], (fx, fy))
            # Keep the task-book reach metric tied to the operator's requested goal, but do
            # not stop observing there. A snapped path deliberately ends elsewhere and may
            # pass through this radius en route; ending the trial here hid later collisions
            # and leaked the still-moving rover into the next trial.
            if math.hypot(fx - gx, fy - gy) < REACH_TOLERANCE:
                reached = True
            with self.lock:
                aborted = self.status == "aborted"
                # Planner completion classifies the active path as a task route (rather
                # than a recovery manoeuvre); follower completion proves the controller has
                # actually stopped on that path. Require both so neither status can race the
                # other and leak a residual command into the next trial.
                mission_done = (self.planner_goal_reached and
                                self.follower_goal_reached)
            if mission_done:
                break
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
            path_len = self.plan_path_len
            chord = self.plan_chord
            path_seen = self.path_seen
            profile_ok = (self.profile_ok and self.atomic_profile_ok and
                          not self.atomic_profile_invalid)
            angular_profile_seen = self.angular_profile_seen
            follower_stale = self.follower_stale
            follower_goal_reached = self.follower_goal_reached
            planner_goal_reached = self.planner_goal_reached
            diagnostics = dict(self.diag)
        # Relative, so legs of different lengths are comparable. Undefined when no route
        # for this goal was captured, or when it was a turn on the spot.
        detour = ((path_len - chord) / chord
                  if path_len is not None and chord > 1e-3 else float("nan"))
        return {
            "goal": (gx, gy),
            "start": (x0, y0),
            "final": (final.pose.pose.position.x, final.pose.pose.position.y,
                      _yaw_of(final.pose.pose.orientation)),
            "reached": reached,
            "planned": any(s.startswith("success") for s in statuses),
            "statuses": statuses,
            "anomaly": any(s not in NOMINAL_STATUSES for s in statuses),
            # Any path at all, including recovery manoeuvres: this feeds 输出达标率, which
            # is about the follower's (v, w)-per-pose invariant, not about route quality.
            "produced_path": path_seen,
            "profile_ok": profile_ok,
            "angular_profile_seen": angular_profile_seen,
            "follower_stale": follower_stale,
            "follower_goal_reached": follower_goal_reached,
            "planner_goal_reached": planner_goal_reached,
            "diagnostics": diagnostics,
            "latency": latency,
            "plan_ms": plan_ms,
            "path_len": path_len if path_len is not None else float("nan"),
            "chord": chord if chord is not None else float("nan"),
            "detour": detour,
            "driven": driven,
            "recovery_events": self._counter("recovery_events") - events0,
            "recovery_successes": self._counter("recovery_successes") - successes0,
            "recovery_aborts": self._counter("recovery_aborts") - aborts0,
            "rmse": math.sqrt(sq_err / n_err) if n_err else float("nan"),
            "clearance_mean": statistics.mean(clearances) if clearances else float("nan"),
            "clearance_min": min(clearances) if clearances else float("nan"),
            "collided": bool(clearances) and min(clearances) < 0.0,
            "measured_clearance": bool(clearances),
            # Where the body came closest and to what, so a collision failure can name the
            # rock instead of only counting trials.
            "worst_at": list(worst[2]) if worst else None,
            "worst_obstacle": list(worst[1]) if worst else None,
            "moved": math.hypot(final.pose.pose.position.x - x0,
                                final.pose.pose.position.y - y0),
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
        diagnostic_turn = bool(rospy.get_param("~diagnostic_turn", False))
        diagnostic_hard_goal = bool(rospy.get_param("~diagnostic_hard_goal", False))
        if diagnostic_hard_goal:
            self.assertEqual(self.scenario, "mixed",
                             "diagnostic_hard_goal requires scenario:=mixed")
            trial = self._run_trial(*HARD_GOALS[0], 35.0)
        elif diagnostic_turn:
            # Match the first failing mixed-tour corner: the previous leg is accepted
            # about 0.5 m before its exact goal, so the next southbound goal is also
            # offset east of the rover instead of lying exactly on its lateral axis.
            trial = self._run_trial(initial.pose.pose.position.x+0.5,
                                    initial.pose.pose.position.y-3.0,
                                    -math.pi/2.0, 30.0)
        else:
            trial = self._run_trial(initial.pose.pose.position.x + 2.0,
                                    initial.pose.pose.position.y, 0.0, 35.0)
        n_poses, n_vel = self._await_consistent_profile()
        trajectory = rospy.wait_for_message("/lunar_planner/trajectory",
                                            LunarTrajectory, timeout=10)

        rospy.loginfo("=== pipeline metrics [mode=%s scenario=%s] ===", mode, self.scenario)
        rospy.loginfo("  moved            = %.3f m", trial["moved"])
        rospy.loginfo("  final goal error = %.3f m", trial["final_err"])
        rospy.loginfo("  final pose       = (%.3f, %.3f, %.3f)", *trial["final"])
        rospy.loginfo("  planner statuses = %s", ",".join(sorted(trial["statuses"])))
        rospy.loginfo("  planner diag     = %s", trial["diagnostics"])
        rospy.loginfo("  tracking RMSE    = %.3f m", trial["rmse"])
        rospy.loginfo("  clearance min   = %.3f m", trial["clearance_min"])
        rospy.loginfo("  plan latency     = %.3f s", trial["latency"])
        rospy.loginfo("  path poses / vel = %d / %d", n_poses, n_vel)

        self.assertGreater(trial["moved"], 1.0)
        self.assertFalse(
            trial["collided"],
            "closed-loop diagnostic put the body inside a ground-truth hazard: "
            "clearance=%.3f at %s against %s" %
            (trial["clearance_min"], trial["worst_at"], trial["worst_obstacle"]))
        if diagnostic_hard_goal:
            # This goal is deliberately inside the raw body+clearance envelope. The safe
            # outcome is a snapped endpoint, so distance to the operator's original click
            # may exceed the ordinary reach tolerance even when execution is correct.
            self.assertTrue(trial["planned"], "hard-goal diagnostic produced no nominal plan")
            self.assertNotIn("aborted", trial["statuses"],
                             "first hard goal should be solved by snapping, not aborted")
            self.assertTrue(trial["follower_goal_reached"],
                            "snapped trajectory never reached its actual endpoint")
            self.assertTrue(trial["planner_goal_reached"],
                            "planner never confirmed snapped mission completion")
            self.assertLess(trial["final_err"], 1.5,
                            "snapped endpoint exceeded max_snap_distance")
        else:
            self.assertLess(trial["final_err"], 0.5)
        # Required in both modes: 3.2 lists the desired linear and angular velocity as a
        # planner output, and the arc mode is the shipping default.
        self.assertGreater(n_poses, 0, "planner published no path")
        self.assertEqual(n_vel, 2 * n_poses,
                         "velocity profile must hold one (v, w) pair per pose")
        self.assertGreater(len(trajectory.path.poses), 0,
                           "planner published no atomic trajectory")
        self.assertEqual(len(trajectory.twists), len(trajectory.path.poses),
                         "atomic trajectory must hold one twist per pose")
        self.assertFalse(trial["follower_stale"],
                         "follower stopped on a nominal GroundGrid publication interval")

    def test_metrics_over_trials(self):
        if (bool(rospy.get_param("~diagnostic_turn", False)) or
                bool(rospy.get_param("~diagnostic_straight", False)) or
                bool(rospy.get_param("~diagnostic_hard_goal", False))):
            self.skipTest("short controller/safety diagnostic requested")
        n_trials = int(rospy.get_param("~n_trials", 3))
        spec = SCENARIOS[self.scenario]
        rospy.wait_for_message("/terrain/costmap", OccupancyGrid, timeout=20)

        tour, hard = [], []
        for _ in range(n_trials):
            for gx, gy, gyaw in spec["tour"]:
                tour.append(self._run_trial(gx, gy, gyaw, spec["timeout"]))
        for gx, gy, gyaw in spec.get("hard", []):
            hard.append(self._run_trial(gx, gy, gyaw, spec.get("hard_timeout", 35.0)))

        every = tour + hard
        report = self._report(every, tour, hard)
        self._dump(report, every)
        self._assert(report, tour, every)

    def _report(self, every, tour, hard):
        """Compute and log every task-book metric; returns the JSON-serialisable summary."""
        tour_rate = sum(1 for t in tour if t["planned"]) / len(tour)
        # Goals with no solution are scored separately, by assertion. See UNREACHABLE_GOALS.
        solvable = [t for t in every if tuple(t["goal"]) not in UNREACHABLE_GOALS]
        overall_rate = sum(1 for t in solvable if t["planned"]) / len(solvable)
        events = sum(t["recovery_events"] for t in every)
        successes = sum(t["recovery_successes"] for t in every)
        aborts = sum(t["recovery_aborts"] for t in every)
        # An entry that ends in an abort was a goal with no solution, so it is neither a
        # recovery nor a failed recovery and does not belong in the denominator. Aborting
        # freely cannot game this: every abort also costs a 规划成功率 trial, and both
        # numbers are reported side by side.
        recoverable = events - aborts
        recovery_rate = successes / recoverable if recoverable else float("nan")
        # 避障成功率 is deliberately not the same question as 近障恢复率: it asks whether
        # the trial got where it was going without ever putting the body inside a hazard,
        # and it is only meaningful where clearance was actually measurable.
        measured = [t for t in every if t["measured_clearance"]]
        collisions = [t for t in measured if t["collided"]]
        # A goal the planner correctly declared unreachable never became an avoidance
        # attempt, so it does not belong in the denominator -- same argument as
        # `recoverable` above. Without this the metric read exactly `reach_rate` (0.75 for
        # both, with zero collisions), i.e. it was measuring how hard the hard goals are.
        # A trial that collided stays in regardless of how it ended: aborting after driving
        # through a rock must not erase the collision.
        attempted = [t for t in measured
                     if t["collided"] or t["reached"] or "aborted" not in t["statuses"]]
        avoid_rate = (sum(1 for t in attempted if not t["collided"]) / len(attempted)
                      if attempted else float("nan"))

        with self.lock:
            cpu = list(self.cpu_pct)
            rss = list(self.rss_mb)
        cpu_over = (sum(1 for c in cpu if c > self.cpu_budget) / len(cpu)) if cpu else float("nan")
        anomaly_rate = sum(1 for t in every if t["anomaly"]) / len(every)
        # Conditioned on a path having been published: a correctly aborted goal produces
        # no output, and counting that as non-conforming would measure goal difficulty
        # instead of the follower invariant this is here to protect.
        emitted = [t for t in every if t["produced_path"]]
        conformance = (sum(1 for t in emitted if t["profile_ok"]) / len(emitted)
                       if emitted else float("nan"))
        stale_rate = sum(1 for t in every if t["follower_stale"]) / len(every)
        angular_coverage = (sum(1 for t in emitted if t["angular_profile_seen"]) / len(emitted)
                            if emitted else float("nan"))

        metrics = {
            "plan_ms": _stats([ms for t in every for ms in t["plan_ms"]]),
            "detour_ratio": _stats([t["detour"] for t in every]),
            "path_len_m": _stats([t["path_len"] for t in every]),
            "driven_m": _stats([t["driven"] for t in every]),
            "tracking_rmse_m": _stats([t["rmse"] for t in every]),
            "clearance_mean_m": _stats([t["clearance_mean"] for t in every]),
            "clearance_min_m": _stats([t["clearance_min"] for t in every]),
            "cpu_pct": _stats(cpu),
            "rss_mb": _stats(rss),
        }
        rates = {
            "plan_success_tour": tour_rate,
            "plan_success_all": overall_rate,
            "reach_tour": sum(1 for t in tour if t["reached"]) / len(tour),
            "reach_rate": sum(1 for t in every if t["reached"]) / len(every),
            "obstacle_avoidance": avoid_rate,
            "near_obstacle_recovery": recovery_rate,
            "cpu_overrun": cpu_over,
            "anomaly": anomaly_rate,
            "output_conformance": conformance,
            "follower_stale": stale_rate,
            "angular_profile_coverage": angular_coverage,
        }
        counts = {"trials": len(every), "tour": len(tour), "hard": len(hard),
                  "solvable": len(solvable),
                  "recovery_events": events, "recovery_successes": successes,
                  "recovery_aborts": aborts, "collisions": len(collisions),
                  "clearance_measured": len(measured),
                  "avoidance_attempts": len(attempted), "paths_emitted": len(emitted)}
        # Carried in the summary itself (not just the per-trial dump) so the assertion
        # message can name the offending goal and rock on the console.
        collided = [{"goal": list(t["goal"]), "clearance_min_m": t["clearance_min"],
                     "at": t["worst_at"], "obstacle": t["worst_obstacle"],
                     "statuses": sorted(t["statuses"])} for t in collisions]

        rospy.loginfo("=== planner metrics [scenario=%s] over %d trials ===",
                      self.scenario, len(every))
        rospy.loginfo("  规划成功率 tour   = %.3f (%d/%d)", tour_rate,
                      sum(1 for t in tour if t["planned"]), len(tour))
        rospy.loginfo("  规划成功率 all    = %.3f (%d/%d solvable; %d unsolvable by design)",
                      overall_rate, sum(1 for t in solvable if t["planned"]), len(solvable),
                      len(every) - len(solvable))
        rospy.loginfo("  reached goal      = %d/%d", sum(1 for t in every if t["reached"]),
                      len(every))
        rospy.loginfo("  reached tour      = %.3f (%d/%d)", rates["reach_tour"],
                      sum(1 for t in tour if t["reached"]), len(tour))
        rospy.loginfo("  避障成功率        = %.3f (%d collision-free / %d attempts; "
                      "%d measured, %d correctly aborted)", avoid_rate,
                      sum(1 for t in attempted if not t["collided"]), len(attempted),
                      len(measured), len(measured) - len(attempted))
        rospy.loginfo("  近障恢复率        = %.3f (%d recovered / %d recoverable; "
                      "%d entered, %d aborted as unreachable)",
                      recovery_rate, successes, recoverable, events, aborts)
        rospy.loginfo("  资源占用超标率    = %.3f (cpu_pct > %.0f%%, %d samples)",
                      cpu_over, self.cpu_budget, len(cpu))
        rospy.loginfo("  异常发生率        = %.3f", anomaly_rate)
        rospy.loginfo("  输出达标率        = %.3f (%d/%d trials that emitted a path)",
                      conformance, sum(1 for t in emitted if t["profile_ok"]), len(emitted))
        rospy.loginfo("  跟踪陈旧停止率    = %.3f", stale_rate)
        rospy.loginfo("  角速度剖面覆盖率  = %.3f", angular_coverage)
        for label, key in (("规划耗时 (ms)", "plan_ms"),
                           ("路径长度偏差", "detour_ratio"),
                           ("轨迹跟踪误差 (m)", "tracking_rmse_m"),
                           ("安全距离均值 (m)", "clearance_mean_m"),
                           ("安全距离最小 (m)", "clearance_min_m"),
                           ("CPU (%)", "cpu_pct"),
                           ("RSS (MB)", "rss_mb")):
            s = metrics[key]
            if s["n"]:
                rospy.loginfo("  %-18s mean/sd/min/max = %.3f / %.3f / %.3f / %.3f (n=%d)",
                              label, s["mean"], s["sd"], s["min"], s["max"], s["n"])
            else:
                rospy.loginfo("  %-18s no samples", label)
        for t in every:
            rospy.loginfo("    goal (%.1f, %.1f) planned=%s reached=%s clr_min=%.2f "
                          "statuses=%s", t["goal"][0], t["goal"][1], t["planned"],
                          t["reached"], t["clearance_min"], ",".join(sorted(t["statuses"])))
        return {"scenario": self.scenario, "counts": counts, "rates": rates,
                "metrics": metrics, "collided": collided,
                "cpu_budget_pct": self.cpu_budget}

    def _dump(self, report, every):
        """Persist the run so the numbers survive as a citable artefact.

        rostest captures loginfo into ~/.ros/log/ rather than the console, which has
        repeatedly meant the metric table went unread; the JSON is what the summary
        script and the report consume.
        """
        path = rospy.get_param("~metrics_out", "")
        if not path:
            path = os.path.expanduser("~/.ros/planner_metrics_%s.json" % self.scenario)
        report = dict(report)
        report["trials"] = [
            {"goal": list(t["goal"]), "start": list(t["start"]),
             "final": list(t["final"]), "reached": t["reached"],
             "planned": t["planned"], "anomaly": t["anomaly"], "collided": t["collided"],
             "produced_path": t["produced_path"], "profile_ok": t["profile_ok"],
             "angular_profile_seen": t["angular_profile_seen"],
             "follower_stale": t["follower_stale"],
             "follower_goal_reached": t["follower_goal_reached"],
             "planner_goal_reached": t["planner_goal_reached"],
             "statuses": sorted(t["statuses"]),
             "latency_s": t["latency"], "path_len_m": t["path_len"],
             "chord_m": t["chord"], "detour_ratio": t["detour"],
             "driven_m": t["driven"], "rmse_m": t["rmse"],
             "clearance_mean_m": t["clearance_mean"], "clearance_min_m": t["clearance_min"],
             "plan_ms": t["plan_ms"]}
            for t in every]
        try:
            directory = os.path.dirname(path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(path, "w") as handle:
                # NaN is not valid JSON but every reader in this repo is Python, and the
                # alternative -- null -- would be indistinguishable from "not collected".
                json.dump(report, handle, indent=2, sort_keys=True)
            rospy.loginfo("  metrics written to %s", path)
        except OSError as exc:
            rospy.logwarn("could not write metrics to %s: %s", path, exc)

    def _assert(self, report, tour, every):
        rates, metrics = report["rates"], report["metrics"]
        # Never allowed anywhere: driving the body through a ground-truth hazard.
        # rostest only echoes the assertion message, so the offenders go in it.
        self.assertEqual(report["counts"]["collisions"], 0,
                         "%d trial(s) put the body inside a hazard footprint:\n%s"
                         % (report["counts"]["collisions"],
                            "\n".join(_describe_collision(c) for c in report["collided"])))
        if self.scenario in STRICT_SCENARIOS:
            self.assertGreaterEqual(rates["plan_success_tour"], 0.99,
                                    "规划成功率 on open terrain")
            self.assertGreaterEqual(rates["plan_success_all"], 0.90,
                                    "规划成功率 including hard goals")
            self.assertGreaterEqual(rates["reach_tour"], 0.99,
                                    "open-terrain tour goals must actually be reached")
            # Guards the failure mode the tiering could otherwise hide: a planner that
            # declares reachable goals unreachable. Open terrain must never abort.
            self.assertFalse([t for t in tour if "aborted" in t["statuses"]],
                             "an open-terrain goal was aborted")
            # And the converse, now that these goals no longer sit in the rate: a goal
            # inside a 40 deg crater wall must be refused, not driven to. Reaching one
            # would mean the slope limit is not being enforced.
            for t in every:
                if tuple(t["goal"]) in UNREACHABLE_GOALS:
                    self.assertFalse(t["reached"],
                                     "goal (%.1f, %.1f) has no legal solution but the rover "
                                     "reached it: the slope limit is not being enforced"
                                     % t["goal"])
                    self.assertIn("aborted", t["statuses"],
                                  "goal (%.1f, %.1f) has no solution; the planner should "
                                  "escalate to Abort rather than retry, got [%s]"
                                  % (t["goal"][0], t["goal"][1],
                                     ",".join(sorted(t["statuses"]))))
        else:
            rospy.logwarn("scenario '%s' is a marginal terrain class: 规划成功率 %.3f "
                          "reported, not asserted", self.scenario,
                          rates["plan_success_tour"])

        self.assertGreater(metrics["plan_ms"]["n"], 0,
                           "no plan_ms samples: /lunar_planner/diagnostics is silent")
        self.assertLess(metrics["plan_ms"]["mean"], 1000.0, "规划耗时 mean exceeds the budget")
        self.assertLess(metrics["plan_ms"]["max"], 1200.0, "规划耗时 worst case exceeds the budget")

        if report["counts"]["recovery_events"] - report["counts"]["recovery_aborts"]:
            self.assertGreaterEqual(rates["near_obstacle_recovery"], 0.8, "近障恢复率")
        else:
            rospy.logwarn("近障恢复率 not asserted: no recoverable entries in this run")
        if metrics["cpu_pct"]["n"]:
            self.assertLess(rates["cpu_overrun"], 0.2,
                            "资源占用超标率: cpu_pct over %.0f%% in %.0f%% of samples"
                            % (self.cpu_budget, 100.0 * rates["cpu_overrun"]))
        else:
            rospy.logwarn("资源占用超标率 not asserted: planner reported no /proc samples")
        self.assertGreater(report["counts"]["paths_emitted"], 0,
                           "planner never published a path in any trial")
        self.assertEqual(rates["output_conformance"], 1.0,
                         "输出达标率: every emitted path must have an equal-length, finite "
                         "legacy profile and atomic trajectory")
        self.assertEqual(rates["follower_stale"], 0.0,
                         "follower stopped because a nominal trajectory or terrain map was stale")
        self.assertGreater(rates["angular_profile_coverage"], 0.0,
                           "planner never emitted a non-zero angular velocity profile")


if __name__ == "__main__":
    rospy.init_node("lunar_pipeline_test")
    rostest.rosrun("groundgrid", "lunar_pipeline_test", LunarPipelineTest)
