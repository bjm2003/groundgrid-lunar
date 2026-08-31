#!/usr/bin/env python3
"""Unit tests for the ROS-free skid-steer identification core."""

import math
import pathlib
import sys
import unittest

import numpy as np

PYTHON_SRC = pathlib.Path(__file__).resolve().parents[1]/"src"
sys.path.insert(0, str(PYTHON_SRC))

from groundgrid.lunar_terrain import AnalyticLunarTerrain, SCENARIOS  # noqa: E402
from groundgrid.skidsteer_identification_core import fit_skidsteer  # noqa: E402


TRUTH = {
    "x_icr": 0.15,
    "alpha_v": 0.90,
    "alpha_w": 0.85,
    "slope_slip_gain": 0.30,
    "slope_grade_gain": 0.40,
}


def gradient(x, y):
    # Varies in both axes so ICR and downslope drift remain independently observable.
    return 0.10+0.015*x, -0.08+0.012*y


def legacy_terrain_arrays(scenario, x, y):
    """Pre-extraction array implementation retained as a regression oracle."""
    tilt, sine_amp, sine_k = scenario["base"]
    ripple_amp, ripple_kx, ripple_ky = scenario["ripple"]
    height = (tilt*x+sine_amp*np.sin(sine_k*y)+
              ripple_amp*np.sin(ripple_kx*x)*np.sin(ripple_ky*y))
    for cx, cy, radius, depth, rim_height, sharpness, _hazard in scenario["craters"]:
        reach = 3.0*radius+2.0
        near = (np.abs(x-cx) < reach) & (np.abs(y-cy) < reach)
        if not near.any():
            continue
        distance = np.hypot(x[near]-cx, y[near]-cy)
        height[near] -= depth*np.exp(-(distance/radius)**sharpness)
        if rim_height:
            height[near] += rim_height*np.exp(
                -((distance-(radius+0.6))/0.65)**2)
    return height


def synthetic_samples(include_invalid=False):
    dt = 0.02
    x, y, yaw, stamp = -2.0, -1.0, 0.2, 0.0
    samples = []
    schedule = [
        (0.5, 0.0, 120), (1.0, 0.0, 120),
        (0.0, 0.4, 120), (0.0, -0.6, 120),
        (0.7, 0.35, 180), (0.8, -0.45, 180),
    ]
    for v_cmd, w_cmd, count in schedule:
        if include_invalid:
            samples.append((stamp, x, y, yaw, float("nan"), float("nan")))
        for _ in range(count):
            samples.append((stamp, x, y, yaw, v_cmd, w_cmd))
            gx, gy = gradient(x, y)
            c, s = math.cos(yaw), math.sin(yaw)
            grad_long = gx*c+gy*s
            grad_lat = -gx*s+gy*c
            vx = TRUTH["alpha_v"]*v_cmd*(
                1.0-TRUTH["slope_grade_gain"]*max(0.0, grad_long))
            omega = TRUTH["alpha_w"]*w_cmd
            vy = -TRUTH["x_icr"]*omega-TRUTH["slope_slip_gain"]*grad_lat
            x += (c*vx-s*vy)*dt
            y += (s*vx+c*vy)*dt
            yaw = math.atan2(math.sin(yaw+omega*dt), math.cos(yaw+omega*dt))
            stamp += dt
        if include_invalid:
            samples.append((stamp, x, y, yaw, float("nan"), float("nan")))
    samples.append((stamp, x, y, yaw, 0.0, 0.0))
    return samples


class IdentificationCoreTest(unittest.TestCase):
    def test_recovers_known_parameters(self):
        result = fit_skidsteer(synthetic_samples(), gradient)
        self.assertLess(max(abs(result[key]-TRUTH[key]) for key in TRUTH), 0.02)

    def test_settle_samples_are_ignored(self):
        reference = fit_skidsteer(synthetic_samples(), gradient)
        result = fit_skidsteer(synthetic_samples(include_invalid=True), gradient)
        for key in TRUTH:
            self.assertAlmostEqual(result[key], reference[key], places=10)

    def test_degenerate_excitation_fails(self):
        samples = [(0.1*i, 0.0, 0.0, 0.0, 0.0, 0.0) for i in range(30)]
        with self.assertRaises(ValueError):
            fit_skidsteer(samples, gradient)

    def test_analytic_terrain_is_vectorised_and_finite(self):
        terrain = AnalyticLunarTerrain(SCENARIOS["mixed"])
        x = np.array([-10.0, 0.0, 5.0])
        y = np.array([-6.0, -2.0, 2.0])
        self.assertTrue(np.all(np.isfinite(terrain.ground_height(x, y))))
        self.assertTrue(math.isfinite(terrain.terrain(1.0, 2.0)))
        self.assertTrue(math.isfinite(terrain.rocks(0.0, -2.0)))
        self.assertTrue(all(math.isfinite(value)
                            for value in terrain.terrain_gradient(-5.0, -4.0)))

    def test_extracted_terrain_preserves_array_results(self):
        grid = np.linspace(-15.0, 15.0, 81)
        x, y = np.meshgrid(grid, grid)
        for scenario_name in ("mixed", "negative"):
            scenario = SCENARIOS[scenario_name]
            expected = legacy_terrain_arrays(scenario, x.copy(), y.copy())
            actual = AnalyticLunarTerrain(scenario).terrain(x, y)
            self.assertTrue(np.array_equal(actual, expected), scenario_name)


if __name__ == "__main__":
    unittest.main()
