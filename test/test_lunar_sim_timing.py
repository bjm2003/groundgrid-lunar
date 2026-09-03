#!/usr/bin/env python3
"""Execute the production simulator callbacks with deterministic ROS-shaped substitutes.

No ROS transport or scheduler is exercised. Interleaving a dynamics tick during ray
construction proves that cloud geometry and its TF lookup stamp belong to one snapshot.
"""
import ast
import math
import pathlib
import threading
from types import SimpleNamespace as NS
import unittest

import numpy as np


class Stamp:
    def __init__(self, ns):
        self.ns = ns

    def __sub__(self, other):
        return Stamp(self.ns - other.ns)

    def to_sec(self):
        return self.ns / 1e9


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class PointField:
    FLOAT32, UINT16 = 7, 4

    def __init__(self, *args):
        self.args = args


def odometry():
    return NS(header=NS(), pose=NS(pose=NS(position=NS(), orientation=NS())))


class SimulatorTimingTest(unittest.TestCase):
    def setUp(self):
        source = pathlib.Path(__file__).resolve().parents[1] / "scripts/lunar_surface_sim.py"
        node = next(n for n in ast.parse(source.read_text(encoding="utf-8")).body
                    if isinstance(n, ast.ClassDef) and n.name == "LunarSurfaceSim")
        self.clock = Stamp(10_000_000_000)
        self.warnings = []
        namespace = {
            "math": math, "np": np, "threading": threading,
            "rospy": NS(Time=NS(now=lambda: self.clock),
                        logwarn_throttle=lambda *args: self.warnings.append(args)),
            "tf": NS(transformations=NS(quaternion_from_euler=lambda r, p, y:
                                        (0.0, 0.0, math.sin(y/2), math.cos(y/2)))),
            "Odometry": odometry, "Header": NS, "PointField": PointField,
            "point_cloud2": NS(create_cloud=lambda header, fields, points:
                               NS(header=header, fields=fields, points=points)),
        }
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), "exec"), namespace)
        self.sim = namespace["LunarSurfaceSim"].__new__(namespace["LunarSurfaceSim"])
        sim = self.sim
        sim.lock = threading.Lock()
        sim.x, sim.y, sim.yaw = -.2, .3, .25
        sim.v, sim.w = 1.0, .4
        sim.alpha_v = sim.alpha_w = 1.0
        sim.x_icr = sim.slope_slip_gain = sim.slope_grade_gain = 0.0
        sim.last = self.clock
        # A sloping plane gives an exact ray intersection without stochastic map inputs.
        sim.terrain_model = NS(height_at=self.height, ground_height=self.height,
                               terrain_gradient=lambda x, y, eps: (.2, -.1))
        sim.sensor_height, sim.min_range, sim.march_step = 1.0, .1, .3
        sim.cloud_radius, sim.noise_std = 5.0, 0.0
        sim.ray_az = np.array([0.0, .5, -.5])
        sim.ray_tan = np.full(3, -.6)
        sim.ray_ring = np.arange(3, dtype=np.uint16)
        sim.rng = np.random.default_rng(42)
        sim.cloud_pub, sim.odom_pub = Publisher(), Publisher()
        self.transforms = []
        sim.br = NS(sendTransform=lambda *args: self.transforms.append(args))

    @staticmethod
    def height(x, y):
        return .2*x - .1*y

    def advance(self):
        self.clock = Stamp(self.clock.ns + 100_000_000)
        self.sim.update(None)

    def cloud_after_ticks(self, ticks):
        """Inject actual update() calls after publish_cloud() acquires its pose."""
        captured = (self.sim.x, self.sim.y, self.sim.yaw, self.sim.last.ns)
        first = True

        def moving_ground(x, y):
            nonlocal first
            if first:
                first = False
                for _ in range(ticks):
                    self.advance()
            return self.height(x, y)

        self.sim.terrain_model.ground_height = moving_ground
        self.sim.publish_cloud(None)
        return captured, self.sim.cloud_pub.messages[-1]

    def test_cloud_stamp_is_capture_time_not_ray_completion_time(self):
        captured, cloud = self.cloud_after_ticks(4)
        self.assertEqual(self.sim.last.ns, captured[3] + 400_000_000)
        self.assertEqual(len(cloud.points), 3)
        self.assertEqual(cloud.header.frame_id, "velodyne")
        self.assertEqual(cloud.header.stamp.ns, captured[3])

    def test_moving_cloud_geometry_matches_tf_at_its_own_stamp(self):
        captured, cloud = self.cloud_after_ticks(4)
        pose_by_stamp = {captured[3]: captured[:3], self.sim.last.ns:
                         (self.sim.x, self.sim.y, self.sim.yaw)}
        x, y, yaw = pose_by_stamp[cloud.header.stamp.ns]
        errors = []
        for lx, ly, lz, _, _ in cloud.points:
            wx = x + math.cos(yaw)*lx - math.sin(yaw)*ly
            wy = y + math.sin(yaw)*lx + math.cos(yaw)*ly
            wz = self.height(x, y) + self.sim.sensor_height + lz
            errors.append(abs(wz - self.height(wx, wy)))
        self.assertLess(max(errors), 1e-6, "cloud must register on the static surface")

    def test_stationary_geometry_and_schema_are_unchanged(self):
        captured, first = self.cloud_after_ticks(0)
        self.sim.publish_cloud(None)
        second = self.sim.cloud_pub.messages[-1]
        self.assertEqual(first.header.stamp.ns, captured[3])
        self.assertEqual(first.points, second.points)
        self.assertEqual([f.args[0] for f in first.fields], ["x", "y", "z", "intensity", "ring"])

    def test_dynamics_stamp_does_not_change_before_pose_lock(self):
        sim = self.sim
        initial = (sim.x, sim.y, sim.yaw, sim.last.ns)
        before_lock = []
        real_lock = sim.lock

        class ObservedLock:
            def __enter__(self):
                before_lock.append((sim.x, sim.y, sim.yaw, sim.last.ns))
                real_lock.acquire()

            def __exit__(self, *_args):
                real_lock.release()

        sim.lock = ObservedLock()
        self.advance()
        self.assertEqual(before_lock, [initial], "stamp and pose must commit under the same lock")
        self.assertAlmostEqual(sim.x, initial[0] + .1*math.cos(initial[2]))
        self.assertAlmostEqual(sim.y, initial[1] + .1*math.sin(initial[2]))
        self.assertAlmostEqual(sim.yaw, initial[2] + .04)
        odom = sim.odom_pub.messages[-1]
        base_tf = next(t for t in self.transforms if t[3] == "base_link")
        self.assertEqual(odom.header.stamp.ns, sim.last.ns)
        self.assertEqual(base_tf[2].ns, sim.last.ns)
        self.assertEqual(base_tf[0], (sim.x, sim.y, self.height(sim.x, sim.y)))

    def test_empty_scan_publishes_no_cloud(self):
        self.sim.ray_tan = np.full(3, 2.0)
        self.sim.publish_cloud(None)
        self.assertEqual(self.sim.cloud_pub.messages, [])
        self.assertEqual(len(self.warnings), 1)


if __name__ == "__main__":
    unittest.main()
