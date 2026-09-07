"""ROS-free attitude of the 2.5-D terrain-following simulator and its rigid LiDAR.

The planar skid-steer integrator still uses map xy velocity and yaw. This supplies
the missing roll/pitch of the same terrain support pose, not a suspension model.
"""
import math

import numpy as np


def terrain_attitude(gx, gy, yaw):
    """Return xyzw quaternion and base-to-map rotation for a tangent support plane.

    Base z is the upward terrain normal (-gx, -gy, 1); the horizontal projection
    of base x retains the integrator's yaw. Both TF and ray casting use this frame.
    """
    if not all(math.isfinite(value) for value in (gx, gy, yaw)):
        raise ValueError("non-finite terrain attitude")
    c, s = math.cos(yaw), math.sin(yaw)
    longitudinal = gx*c + gy*s
    lateral = -gx*s + gy*c
    pitch = -math.atan(longitudinal)
    roll = math.atan(lateral*math.cos(pitch))
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rotation = np.array([[c*cp, c*sp*sr-s*cr, c*sp*cr+s*sr],
                         [s*cp, s*sp*sr+c*cr, s*sp*cr-c*sr],
                         [-sp, cp*sr, cp*cr]])
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    quaternion = (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                  cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
    return quaternion, rotation
