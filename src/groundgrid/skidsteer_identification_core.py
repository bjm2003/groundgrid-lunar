"""ROS-free least-squares core for skid-steer parameter identification."""

import math

import numpy as np


PARAMETER_NAMES = (
    "x_icr",
    "alpha_v",
    "alpha_w",
    "slope_slip_gain",
    "slope_grade_gain",
)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _solve_two_column(rows, values, label):
    if len(rows) < 2:
        raise ValueError("insufficient %s excitation (%d usable intervals)" %
                         (label, len(rows)))
    matrix = np.asarray(rows, dtype=float)
    vector = np.asarray(values, dtype=float)
    solution, _residuals, rank, singular = np.linalg.lstsq(matrix, vector, rcond=None)
    ill_conditioned = (len(singular) < 2 or singular[-1] <= 0.0 or
                       singular[0]/singular[-1] > 1e8)
    if rank < 2 or ill_conditioned or not np.all(np.isfinite(solution)):
        raise ValueError("degenerate %s identification matrix (rank=%d)" % (label, rank))
    return float(solution[0]), float(solution[1])


def fit_skidsteer(samples, gradient_fn):
    """Fit model parameters from (t,x,y,yaw,v_cmd,w_cmd) samples.

    Intervals containing NaN commands are the explicit settle-window representation and
    are ignored. The gradient function must return dz/dx and dz/dy at a world point.
    """
    if len(samples) < 20:
        raise ValueError("not enough odometry samples (%d)" % len(samples))

    w_num = w_den = 0.0
    lateral_rows, lateral_values = [], []
    forward_rows, forward_values = [], []
    for first, second in zip(samples, samples[1:]):
        t0, x0, y0, yaw0, v0, w0 = first
        t1, x1, y1, yaw1, v1, w1 = second
        values = (t0, x0, y0, yaw0, v0, w0, t1, x1, y1, yaw1, v1, w1)
        if not all(math.isfinite(value) for value in values):
            continue
        dt = t1-t0
        if dt <= 1e-4 or v0 != v1 or w0 != w1:
            continue
        world_vx = (x1-x0)/dt
        world_vy = (y1-y0)/dt
        mid_yaw = yaw0+0.5*_wrap(yaw1-yaw0)
        c, s = math.cos(mid_yaw), math.sin(mid_yaw)
        vx = c*world_vx+s*world_vy
        vy = -s*world_vx+c*world_vy
        omega = _wrap(yaw1-yaw0)/dt
        gx, gy = gradient_fn(0.5*(x0+x1), 0.5*(y0+y1))
        if not math.isfinite(gx) or not math.isfinite(gy):
            continue
        grad_long = gx*c+gy*s
        grad_lat = -gx*s+gy*c

        if abs(w0) > 1e-3:
            w_num += omega*w0
            w_den += w0*w0
            lateral_rows.append([-omega, -grad_lat])
            lateral_values.append(vy)
        if abs(v0) > 1e-3:
            forward_rows.append([v0, -v0*max(0.0, grad_long)])
            forward_values.append(vx)

    if w_den <= 1e-9:
        raise ValueError("insufficient yaw excitation")
    alpha_w = w_num/w_den
    x_icr, slope_slip_gain = _solve_two_column(
        lateral_rows, lateral_values, "lateral")
    alpha_v, beta = _solve_two_column(
        forward_rows, forward_values, "longitudinal")
    if abs(alpha_v) <= 1e-6:
        raise ValueError("identified alpha_v is singular")
    params = {
        "x_icr": x_icr,
        "alpha_v": alpha_v,
        "alpha_w": alpha_w,
        "slope_slip_gain": slope_slip_gain,
        "slope_grade_gain": beta/alpha_v,
    }
    if not all(math.isfinite(value) for value in params.values()):
        raise ValueError("identification produced a non-finite parameter")
    return params
