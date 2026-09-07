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
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))
from groundgrid.lunar_sensor_geometry import terrain_attitude
from groundgrid.lunar_terrain import AnalyticLunarTerrain, SCENARIOS


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
            "terrain_attitude": terrain_attitude,
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
        _, rotation = terrain_attitude(.2, -.1, yaw)
        for lx, ly, lz, _, _ in cloud.points:
            wx, wy, wz = (np.array([x, y, self.height(x, y)]) +
                          rotation @ np.array([lx, ly, lz + self.sim.sensor_height]))
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

    @staticmethod
    def rotation_of(q):
        """Independent quaternion matrix for checking the actual published TF."""
        x, y, z, w = q
        return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

    def test_published_support_plane_matches_slope_and_odometry(self):
        self.advance()
        base_tf = next(t for t in self.transforms if t[3] == "base_link")
        normal = self.rotation_of(base_tf[1])[:, 2]
        np.testing.assert_allclose(-normal[:2]/normal[2], [.2, -.1], atol=1e-12)
        q = self.sim.odom_pub.messages[-1].pose.pose.orientation
        self.assertEqual((q.x, q.y, q.z, q.w), base_tf[1])
        self.assertAlmostEqual(math.atan2(self.rotation_of(base_tf[1])[1, 0],
                                         self.rotation_of(base_tf[1])[0, 0]), self.sim.yaw)

    def test_attitude_normal_for_forward_reverse_and_cross_slope_headings(self):
        for gx, gy in ((0., 0.), (.2, -.1), (-.4, .3), (.5, .5)):
            for yaw in (0., math.pi/2, math.pi, -2.7):
                q, rotation = terrain_attitude(gx, gy, yaw)
                np.testing.assert_allclose(self.rotation_of(q), rotation, atol=1e-12)
                np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
                np.testing.assert_allclose(-rotation[:2, 2]/rotation[2, 2], [gx, gy], atol=1e-12)
                self.assertAlmostEqual(sum(v*v for v in q), 1.)

    def test_actual_slope_start_has_tangent_not_horizontal_support(self):
        terrain = AnalyticLunarTerrain(SCENARIOS['slope'])
        self.sim.x, self.sim.y, self.sim.yaw = 0., -5.5, 0.
        self.sim.v = self.sim.w = 0.
        self.sim.terrain_model = terrain
        self.advance()
        origin, q, *_ = next(t for t in self.transforms if t[3] == 'base_link')
        normal = self.rotation_of(q)[:, 2]
        # Recorded false-step rejection was near (-1.125,-6.175), inside the empty
        # support region. Check geometry, never bypass its production lethal cost.
        xy = np.array([-1.125, -6.175])
        expected = terrain.height_at(*xy)
        tangent = origin[2] - np.dot(normal[:2], xy-np.array(origin[:2]))/normal[2]
        self.assertGreater(abs(origin[2]-expected), .25)
        self.assertLess(abs(tangent-expected), .02)

    def test_tilted_cloud_matches_broadcast_sensor_transform(self):
        self.advance()
        self.sim.publish_cloud(None)
        origin, q, stamp, *_ = next(t for t in self.transforms if t[3] == 'base_link')
        offset, sensor_q, *_ = next(t for t in self.transforms if t[3] == 'velodyne')
        cloud = self.sim.cloud_pub.messages[-1]
        self.assertEqual(stamp.ns, cloud.header.stamp.ns)
        self.assertEqual(sensor_q, (0, 0, 0, 1))
        rotation = self.rotation_of(q)
        sensor = np.array(origin) + rotation @ np.array(offset)
        self.assertGreater(np.linalg.norm(sensor[:2]-origin[:2]), .1)
        for point in cloud.points:
            x, y, z = sensor + rotation @ np.array(point[:3])
            self.assertAlmostEqual(z, self.height(x, y), places=6)

    def test_level_ground_keeps_original_ray_intersections(self):
        self.sim.terrain_model = NS(height_at=lambda x, y: 0.,
                                    ground_height=lambda x, y: np.zeros_like(x),
                                    terrain_gradient=lambda *args: (0., 0.))
        self.sim.publish_cloud(None)
        for i, point in enumerate(self.sim.cloud_pub.messages[-1].points):
            rho = -self.sim.sensor_height/self.sim.ray_tan[i]
            np.testing.assert_allclose(point[:3], [rho*math.cos(self.sim.ray_az[i]),
                                      rho*math.sin(self.sim.ray_az[i]), -self.sim.sensor_height], atol=1e-6)

    def test_tilted_rays_keep_first_obstacle_return(self):
        # A nearer parallel surface represents an occluder. Returned points must lie
        # on it, not on the hidden support plane used to orient the rover.
        self.sim.terrain_model.ground_height = lambda x, y: self.height(x, y) + .3
        self.sim.publish_cloud(None)
        _, rotation = terrain_attitude(.2, -.1, self.sim.yaw)
        origin = np.array([self.sim.x, self.sim.y, self.height(self.sim.x, self.sim.y)])
        for point in self.sim.cloud_pub.messages[-1].points:
            local = np.array(point[:3]) + np.array([0, 0, self.sim.sensor_height])
            x, y, z = origin + rotation @ local
            self.assertAlmostEqual(z-self.height(x, y), .3, places=6)

    def test_nonfinite_attitude_is_rejected(self):
        for args in ((float('nan'), 0., 0.), (0., float('inf'), 0.), (0., 0., float('nan'))):
            with self.assertRaises(ValueError):
                terrain_attitude(*args)


if __name__ == "__main__":
    unittest.main()
