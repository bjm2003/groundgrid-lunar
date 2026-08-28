#!/usr/bin/env python3
"""ROS-free analytic lunar terrain shared by simulation and identification."""

import numpy as np


def _dense_rocks():
    """Jittered 4x3 boulder field whose corridors admit the rover footprint.

    The closest centres are 3.52 m apart. After the perception step-height window
    expands each 0.5 m rock to about 0.8 m, the remaining 1.92 m corridor still admits
    the 1.5 m body. A field that could seal shut would measure layout, not planning.
    """
    return [(-6.25, -9.78, 0.5, 0.5), (-5.72, -6.19, 0.5, 0.5),
            (-6.18, -1.75, 0.5, 0.5), (-1.78, -10.24, 0.5, 0.5),
            (-2.21, -5.80, 0.5, 0.5), (-1.73, -2.22, 0.5, 0.5),
            (2.19, -9.75, 0.5, 0.5), (1.76, -6.26, 0.5, 0.5),
            (2.24, -1.79, 0.5, 0.5), (5.75, -10.18, 0.5, 0.5),
            (6.22, -5.77, 0.5, 0.5), (5.80, -2.20, 0.5, 0.5)]


# crater tuple: (x, y, radius, depth, rim_height, sharpness, hazard_radius)
# rock tuple:   (x, y, radius, height)
SCENARIOS = {
    # Original regression terrain. Keep these coefficients unchanged so earlier runs
    # remain comparable.
    "mixed": {
        "base": (0.025, 0.15, 0.12),
        "ripple": (0.05, 0.7, 0.5),
        "craters": [(5.0, 2.0, 3.2, 1.8, 0.55, 4, 2.0)],
        "rocks": [(0.0, -2.0, 0.65, 0.55),
                  (8.0, -5.0, 0.9, 0.75),
                  (-3.0, 5.0, 0.5, 0.40)],
        "start": (-10.0, -6.0, 0.0),
    },
    # Everywhere traversable: the path-length-detour baseline.
    "flat": {
        "base": (0.01, 0.02, 0.10),
        "ripple": (0.03, 0.4, 0.35),
        "craters": [],
        "rocks": [],
        "start": (-10.0, -6.0, 0.0),
    },
    # Dense positive obstacles with a start west of the boulder field.
    "dense": {
        "base": (0.01, 0.03, 0.10),
        "ripple": (0.03, 0.4, 0.35),
        "craters": [],
        "rocks": _dense_rocks(),
        "start": (-11.0, -8.0, 0.0),
    },
    # amp*k = 1.2*0.35 = 0.42 (22.8 deg), straddling the 20 deg slope limit.
    # The shared start would itself be too steep, so this scenario uses a point in the
    # largest passable component.
    "slope": {
        "base": (0.15, 0.30, 0.09),
        "ripple": (1.2, 0.35, 0.28),
        "craters": [],
        "rocks": [],
        "start": (0.0, -5.5, 0.0),
    },
    # Concentrated negative obstacles. The super-Gaussian walls exceed the step limit;
    # the current perception stack detects the unsigned height discontinuity but does not
    # classify pit versus boulder.
    "negative": {
        "base": (0.01, 0.04, 0.10),
        "ripple": (0.03, 0.4, 0.35),
        "craters": [(-4.0, -3.0, 1.5, 1.2, 0.0, 6, 1.5),
                    (-1.0, -6.5, 1.6, 1.2, 0.0, 6, 1.6),
                    (2.0, -3.5, 1.4, 1.0, 0.0, 6, 1.4),
                    (-5.5, -8.5, 1.5, 1.1, 0.0, 6, 1.5),
                    (1.0, -9.0, 1.5, 1.2, 0.0, 6, 1.5)],
        "rocks": [],
        "start": (-10.0, -6.0, 0.0),
    },
}


class AnalyticLunarTerrain:
    """Vectorised terrain model with scalar height/gradient convenience methods."""

    def __init__(self, scenario):
        self.scenario = scenario

    def terrain(self, x, y):
        """Terrain-only height for broadcast-compatible scalar or array inputs.

        Crater corrections are evaluated only within three shape radii plus 2 m. Outside
        that mask their exponential contribution is negligible, while skipping them is
        important to ray-cast simulator performance. Bowl and rim updates remain separate
        and ordered to preserve the original mixed-scenario numerical result.
        """
        x, y = np.broadcast_arrays(np.asarray(x, dtype=float),
                                   np.asarray(y, dtype=float))
        output_shape = x.shape
        x = x.reshape(-1)
        y = y.reshape(-1)
        tilt, sine_amp, sine_k = self.scenario["base"]
        r_amp, r_kx, r_ky = self.scenario["ripple"]
        z = tilt*x + sine_amp*np.sin(sine_k*y) + r_amp*np.sin(r_kx*x)*np.sin(r_ky*y)
        for cx, cy, radius, depth, rim_h, sharpness, _hazard_r in self.scenario["craters"]:
            reach = 3.0*radius + 2.0
            near = (np.abs(x-cx) < reach) & (np.abs(y-cy) < reach)
            if not np.any(near):
                continue
            r = np.hypot(x[near]-cx, y[near]-cy)
            z[near] -= depth*np.exp(-(r/radius)**sharpness)
            if rim_h:
                z[near] += rim_h*np.exp(-((r-(radius+0.6))/0.65)**2)
        if output_shape:
            return z.reshape(output_shape)
        return float(z[0])

    def height_at(self, x, y):
        return float(self.terrain(np.array([x], dtype=float),
                                  np.array([y], dtype=float))[0])

    def rocks(self, x, y):
        x, y = np.broadcast_arrays(np.asarray(x, dtype=float),
                                   np.asarray(y, dtype=float))
        output_shape = x.shape
        x = x.reshape(-1)
        y = y.reshape(-1)
        height = np.zeros_like(x)
        for rx, ry, radius, obstacle_height in self.scenario["rocks"]:
            near = (np.abs(x-rx) < radius) & (np.abs(y-ry) < radius)
            if not np.any(near):
                continue
            distance = np.hypot(x[near]-rx, y[near]-ry)
            height[near] = np.maximum(
                height[near], obstacle_height*(1.0-distance/radius))
        if output_shape:
            return height.reshape(output_shape)
        return float(height[0])

    def ground_height(self, x, y):
        return self.terrain(x, y) + self.rocks(x, y)

    def terrain_gradient(self, x, y, eps=0.05):
        gx = (self.height_at(x+eps, y)-self.height_at(x-eps, y))/(2.0*eps)
        gy = (self.height_at(x, y+eps)-self.height_at(x, y-eps))/(2.0*eps)
        return gx, gy
