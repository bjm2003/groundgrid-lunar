#!/usr/bin/env python3
"""Identify the skid-steer slip model from commanded motion and odometry.

The ROS wrapper owns excitation, settle-window filtering and file output. Numerical
fitting and analytic terrain evaluation live in ROS-free modules so they can be tested
without launching the vehicle stack.
"""

import math
import os
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from groundgrid.lunar_terrain import AnalyticLunarTerrain, SCENARIOS
from groundgrid.skidsteer_identification_core import fit_skidsteer


def yaw_of(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y),
                      1.0-2.0*(q.y*q.y+q.z*q.z))


class Identifier:
    def __init__(self):
        self.lock = threading.Lock()
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            "/localization/odometry/filtered_map", Odometry, self.on_odom, queue_size=50)
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.output = rospy.get_param(
            "~output", os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "config", "skid_steer_model.yaml"))
        self.settle = float(rospy.get_param("~settle", 0.6))
        scenario_name = rospy.get_param(
            "~scenario", rospy.get_param("/lunar_surface_sim/scenario", "mixed"))
        if scenario_name not in SCENARIOS:
            raise ValueError("unknown terrain scenario '%s'" % scenario_name)
        self.terrain = AnalyticLunarTerrain(SCENARIOS[scenario_name])
        self.samples = []
        self.cur_v = 0.0
        self.cur_w = 0.0
        self.valid_after = rospy.Time(0)

        # (v_cmd, w_cmd, duration_s): straights identify alpha_v, turns identify
        # alpha_w/x_icr, and arcs over non-flat ground identify both slope gains.
        self.schedule = [
            (0.0, 0.0, 1.0),
            (0.4, 0.0, 3.0), (0.8, 0.0, 3.0), (1.2, 0.0, 3.0),
            (0.0, 0.4, 3.0), (0.0, -0.4, 3.0),
            (0.0, 0.7, 3.0), (0.0, -0.7, 3.0),
            (0.6, 0.3, 3.0), (0.6, -0.3, 3.0),
            (0.9, 0.5, 3.0), (0.9, -0.5, 3.0),
            (0.0, 0.0, 1.0),
        ]

    def on_odom(self, msg):
        pose = msg.pose.pose
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        with self.lock:
            if stamp < self.valid_after:
                v_cmd = w_cmd = float("nan")
            else:
                v_cmd, w_cmd = self.cur_v, self.cur_w
            self.samples.append((
                stamp.to_sec(), pose.position.x, pose.position.y,
                yaw_of(pose.orientation), v_cmd, w_cmd))

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.sleep(0.5)
        for v_cmd, w_cmd, duration in self.schedule:
            changed_at = rospy.Time.now()
            with self.lock:
                self.cur_v = v_cmd
                self.cur_w = w_cmd
                self.valid_after = changed_at+rospy.Duration(self.settle)
            end = changed_at+rospy.Duration(duration)
            while not rospy.is_shutdown() and rospy.Time.now() < end:
                command = Twist()
                command.linear.x = v_cmd
                command.angular.z = w_cmd
                self.cmd_pub.publish(command)
                rate.sleep()
        with self.lock:
            self.cur_v = 0.0
            self.cur_w = 0.0
            self.valid_after = rospy.Time.now()+rospy.Duration(self.settle)
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.3)
        return self.solve()

    def solve(self):
        with self.lock:
            samples = list(self.samples)
        try:
            params = fit_skidsteer(samples, self.terrain.terrain_gradient)
        except ValueError as error:
            rospy.logerr("identify_skidsteer failed: %s; no parameter file written", error)
            return False
        self.report(params)
        return self.write_yaml(params)

    def report(self, params):
        rospy.loginfo("=== identified skid-steer parameters ===")
        for key, value in params.items():
            rospy.loginfo("  %-16s = % .5f", key, value)
        truth = {
            key: rospy.get_param("/lunar_surface_sim/"+key, None)
            for key in params
        }
        if all(value is not None for value in truth.values()):
            rospy.loginfo("--- recovery error vs injected truth ---")
            worst = 0.0
            for key in params:
                error = abs(params[key]-float(truth[key]))
                worst = max(worst, error)
                rospy.loginfo("  %-16s truth=% .5f  err=% .5f",
                              key, float(truth[key]), error)
            rospy.loginfo("max abs recovery error = %.5f", worst)

    def write_yaml(self, params):
        try:
            path = os.path.abspath(self.output)
            with open(path, "w", encoding="utf-8") as handle:
                handle.write("# Identified by scripts/identify_skidsteer.py. Merge these into\n")
                handle.write("# the skid_steer_model block of config/lunar_system.yaml.\n")
                handle.write("skid_steer_model:\n")
                for key, value in params.items():
                    handle.write("  %s: %.6f\n" % (key, value))
            rospy.loginfo("wrote identified parameters -> %s", path)
            return True
        except OSError as error:
            rospy.logerr("failed to write %s: %s", self.output, error)
            return False


if __name__ == "__main__":
    rospy.init_node("identify_skidsteer")
    try:
        if not Identifier().run():
            sys.exit(1)
    except ValueError as error:
        rospy.logfatal("identify_skidsteer configuration error: %s", error)
        sys.exit(2)
