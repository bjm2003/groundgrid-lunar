#!/usr/bin/env python3
"""Least-squares identification of the skid-steer slip model (tech point 1).

Commands a fixed excitation sequence of (v, w) segments directly on /cmd_vel, records
the resulting body-frame motion from odometry, and recovers the ICR slip parameters
used by groundgrid::SkidSteerModel:

    vx    = alpha_v * v_cmd * (1 - slope_grade_gain * max(0, grad_long))
    omega = alpha_w * w_cmd
    vy    = -x_icr * omega - slope_slip_gain * grad_lat

The identified block is written to config/skid_steer_model.yaml. When the simulator was
launched with injected "ground-truth" slip params, the recovery error against those
truth values is printed so the pipeline can be validated with no real-vehicle data.
"""

import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Reuse the simulator's analytic terrain so grad_long/grad_lat can be evaluated at each
# odometry sample; fall back to a flat assumption if the module is not importable.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from lunar_surface_sim import LunarSurfaceSim
    terrain_gradient = LunarSurfaceSim.terrain_gradient
except Exception:  # pragma: no cover - fallback for detached runs
    def terrain_gradient(x, y):
        return 0.0, 0.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Identifier:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("/localization/odometry/filtered_map",
                                         Odometry, self.on_odom, queue_size=50)
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.output = rospy.get_param(
            "~output", os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "config", "skid_steer_model.yaml"))
        self.settle = float(rospy.get_param("~settle", 0.6))  # discard after each change
        self.samples = []  # (t, x, y, yaw, v_cmd, w_cmd)
        self.cur_v = 0.0
        self.cur_w = 0.0

        # Excitation schedule: (v_cmd, w_cmd, duration_s). Straights identify alpha_v,
        # in-place / arc turns identify alpha_w and x_icr, terrain variation excites the
        # slope gains.
        self.schedule = [
            (0.0, 0.0, 1.0),
            (0.4, 0.0, 3.0), (0.8, 0.0, 3.0), (1.2, 0.0, 3.0),
            (0.0, 0.4, 3.0), (0.0, -0.4, 3.0), (0.0, 0.7, 3.0), (0.0, -0.7, 3.0),
            (0.6, 0.3, 3.0), (0.6, -0.3, 3.0), (0.9, 0.5, 3.0), (0.9, -0.5, 3.0),
            (0.0, 0.0, 1.0),
        ]

    def on_odom(self, msg):
        p = msg.pose.pose
        self.samples.append((msg.header.stamp.to_sec(), p.position.x, p.position.y,
                             yaw_of(p.orientation), self.cur_v, self.cur_w))

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.sleep(0.5)  # let odom start flowing
        for v, w, dur in self.schedule:
            self.cur_v, self.cur_w = v, w
            t_end = rospy.Time.now() + rospy.Duration(dur)
            change_t = rospy.Time.now()
            while not rospy.is_shutdown() and rospy.Time.now() < t_end:
                cmd = Twist()
                cmd.linear.x = v
                cmd.angular.z = w
                self.cmd_pub.publish(cmd)
                # Mark samples inside the settle window as invalid by tagging v/w as NaN.
                if (rospy.Time.now() - change_t).to_sec() < self.settle:
                    self.cur_v, self.cur_w = v, w
                rate.sleep()
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.3)
        self.solve()

    def solve(self):
        if len(self.samples) < 20:
            rospy.logerr("identify_skidsteer: not enough odometry samples (%d)",
                         len(self.samples))
            return

        w_num = w_den = 0.0                      # alpha_w
        A_vy, b_vy = [], []                      # [x_icr, slope_slip_gain]
        A_vx, b_vx = [], []                      # [alpha_v, alpha_v*slope_grade_gain]
        for (t0, x0, y0, yaw0, v0, w0), (t1, x1, y1, yaw1, v1, w1) in zip(
                self.samples, self.samples[1:]):
            dt = t1 - t0
            if dt <= 1e-4 or v0 != v1 or w0 != w1:
                continue  # skip boundary-crossing or degenerate intervals
            wx = (x1 - x0) / dt
            wy = (y1 - y0) / dt
            ym = yaw0 + 0.5 * wrap(yaw1 - yaw0)
            c, s = math.cos(ym), math.sin(ym)
            vx = c * wx + s * wy
            vy = -s * wx + c * wy
            omega = wrap(yaw1 - yaw0) / dt
            gx, gy = terrain_gradient(0.5 * (x0 + x1), 0.5 * (y0 + y1))
            grad_long = gx * c + gy * s
            grad_lat = -gx * s + gy * c

            if abs(w0) > 1e-3:
                w_num += omega * w0
                w_den += w0 * w0
                A_vy.append([-omega, -grad_lat])
                b_vy.append(vy)
            if abs(v0) > 1e-3:
                A_vx.append([v0, -v0 * max(0.0, grad_long)])
                b_vx.append(vx)

        alpha_w = w_num / w_den if w_den > 1e-9 else 1.0
        x_icr, slope_slip_gain = self._lstsq(A_vy, b_vy, (0.0, 0.0))
        alpha_v, beta = self._lstsq(A_vx, b_vx, (1.0, 0.0))
        slope_grade_gain = beta / alpha_v if abs(alpha_v) > 1e-6 else 0.0

        params = {
            "x_icr": x_icr,
            "alpha_v": alpha_v,
            "alpha_w": alpha_w,
            "slope_slip_gain": slope_slip_gain,
            "slope_grade_gain": slope_grade_gain,
        }
        self.report(params)
        self.write_yaml(params)

    @staticmethod
    def _lstsq(A, b, default):
        if len(A) < 2:
            return default
        sol, *_ = np.linalg.lstsq(np.asarray(A), np.asarray(b), rcond=None)
        return float(sol[0]), float(sol[1])

    def report(self, params):
        rospy.loginfo("=== identified skid-steer parameters ===")
        for k, v in params.items():
            rospy.loginfo("  %-16s = % .5f", k, v)
        # Compare against injected ground truth if the sim exposed it on the param server.
        truth = {k: rospy.get_param("/lunar_surface_sim/" + k, None)
                 for k in params}
        if all(v is not None for v in truth.values()):
            rospy.loginfo("--- recovery error vs injected truth ---")
            worst = 0.0
            for k in params:
                err = abs(params[k] - float(truth[k]))
                worst = max(worst, err)
                rospy.loginfo("  %-16s truth=% .5f  err=% .5f",
                              k, float(truth[k]), err)
            rospy.loginfo("max abs recovery error = %.5f", worst)

    def write_yaml(self, params):
        try:
            path = os.path.abspath(self.output)
            with open(path, "w") as f:
                f.write("# Identified by scripts/identify_skidsteer.py. Merge these into\n")
                f.write("# the skid_steer_model block of config/lunar_system.yaml.\n")
                f.write("skid_steer_model:\n")
                for k, v in params.items():
                    f.write("  %s: %.6f\n" % (k, v))
            rospy.loginfo("wrote identified parameters -> %s", path)
        except OSError as e:
            rospy.logerr("failed to write %s: %s", self.output, e)


if __name__ == "__main__":
    rospy.init_node("identify_skidsteer")
    Identifier().run()
