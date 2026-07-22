#!/usr/bin/env python3
"""Closed-loop lunar pipeline test with A/B metrics.

Beyond the original reach-goal check, this records the metrics needed for the
ideal-arc vs. dynamics-primitive comparison (复核复算): planning latency, tracking
RMSE against the published path, and whether a velocity profile is emitted. The mode
label is read from the planner's use_dynamics_primitives param so the same test file
serves both A and B launches.
"""
import math
import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Float32MultiArray, String


class LunarPipelineTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.last_path = None
        self.last_vel = None
        self.status = None
        self.first_success_time = None
        rospy.Subscriber("/lunar_planner/path", Path, self._on_path, queue_size=1)
        rospy.Subscriber("/lunar_planner/velocity_profile", Float32MultiArray,
                         self._on_vel, queue_size=1)
        rospy.Subscriber("/lunar_planner/status", String, self._on_status, queue_size=1)

    def _on_path(self, msg):
        with self.lock:
            self.last_path = msg

    def _on_vel(self, msg):
        with self.lock:
            self.last_vel = msg

    def _on_status(self, msg):
        with self.lock:
            self.status = msg.data
            if msg.data == "success" and self.first_success_time is None:
                self.first_success_time = rospy.Time.now()

    def _cross_track(self, x, y):
        with self.lock:
            path = self.last_path
        if path is None or not path.poses:
            return None
        return min(math.hypot(p.pose.position.x - x, p.pose.position.y - y)
                   for p in path.poses)

    def test_closed_loop(self):
        use_dyn = rospy.get_param("/state_lattice_planner/use_dynamics_primitives", False)
        mode = "dynamics_primitives" if use_dyn else "ideal_arcs"

        cost = rospy.wait_for_message("/terrain/costmap", OccupancyGrid, timeout=20)
        self.assertGreater(sum(1 for v in cost.data if 0 <= v < 100), 1000)
        initial = rospy.wait_for_message("/localization/odometry/filtered_map", Odometry, timeout=5)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = initial.pose.pose.position.x + 2.0
        goal.pose.position.y = initial.pose.pose.position.y
        goal.pose.orientation.w = 1.0
        pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)

        goal_time = rospy.Time.now()
        end = goal_time + rospy.Duration(35)
        rate = rospy.Rate(5)
        final = initial
        sq_err = 0.0
        n_err = 0
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            goal.header.stamp = rospy.Time.now()
            pub.publish(goal)
            final = rospy.wait_for_message("/localization/odometry/filtered_map", Odometry, timeout=2)
            fx, fy = final.pose.pose.position.x, final.pose.pose.position.y
            ct = self._cross_track(fx, fy)
            if ct is not None:
                sq_err += ct * ct
                n_err += 1
            if math.hypot(fx - goal.pose.position.x, fy - goal.pose.position.y) < 0.4:
                break
            rate.sleep()

        moved = math.hypot(final.pose.pose.position.x - initial.pose.pose.position.x,
                           final.pose.pose.position.y - initial.pose.pose.position.y)
        final_err = math.hypot(final.pose.pose.position.x - goal.pose.position.x,
                               final.pose.pose.position.y - goal.pose.position.y)
        rmse = math.sqrt(sq_err / n_err) if n_err else float("nan")
        with self.lock:
            plan_latency = ((self.first_success_time - goal_time).to_sec()
                            if self.first_success_time else float("nan"))
            vel_len = len(self.last_vel.data) if self.last_vel else 0

        rospy.loginfo("=== pipeline metrics [mode=%s] ===", mode)
        rospy.loginfo("  moved            = %.3f m", moved)
        rospy.loginfo("  final goal error = %.3f m", final_err)
        rospy.loginfo("  tracking RMSE    = %.3f m (n=%d)", rmse, n_err)
        rospy.loginfo("  plan latency     = %.3f s", plan_latency)
        rospy.loginfo("  vel profile len  = %d", vel_len)

        self.assertGreater(moved, 1.0)
        self.assertLess(final_err, 0.5)
        # In dynamics mode the planner must emit an aligned (v, w) velocity profile.
        if use_dyn:
            self.assertGreater(vel_len, 0,
                               "dynamics-primitive planner must publish a velocity profile")


if __name__ == "__main__":
    rospy.init_node("lunar_pipeline_test")
    rostest.rosrun("groundgrid", "lunar_pipeline_test", LunarPipelineTest)
