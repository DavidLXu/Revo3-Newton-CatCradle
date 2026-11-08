"""A continuous joint-space spline with one minimum-jerk time envelope.

Spatial IK knots are interpolation samples, not stops. Only the two ends of
the complete path have zero velocity and acceleration. These functions never
read or modify rope state.
"""

import numpy as np
from scipy.interpolate import CubicSpline


def make_joint_path(knots, targets):
    knots, targets = np.asarray(knots), np.asarray(targets)
    spline = CubicSpline(knots, targets, axis=0)
    path = dict(knots=knots.tolist(), coefficients=spline.c.tolist())
    path_arrays(path)
    return path


def path_arrays(path):
    knots = np.asarray(path["knots"], dtype=float)
    coefficients = np.asarray(path["coefficients"], dtype=float)
    if (knots.ndim != 1 or len(knots) < 2 or knots[0] != 0 or knots[-1] != 1
            or np.any(np.diff(knots) <= 0) or not np.isfinite(knots).all()
            or coefficients.ndim != 3
            or coefficients.shape[:2] != (4, len(knots) - 1)
            or not np.isfinite(coefficients).all()):
        raise ValueError("Invalid normalized joint path")
    return knots, coefficients


def evaluate_joint_path(path, progress):
    """Evaluate spatial progress, before applying the global time envelope."""
    knots = np.asarray(path["knots"])
    coefficients = np.asarray(path["coefficients"])
    u = float(np.clip(progress, 0, 1))
    i = int(np.clip(np.searchsorted(knots, u, side="right") - 1, 0, len(knots) - 2))
    x = u - knots[i]
    a, b, c, d = coefficients[:, i]
    return ((a * x + b) * x + c) * x + d


def joint_path_bounds(path, duration):
    """Exact per-coordinate spatial extrema; conservative temporal speed bound.

Each spatial interval's maximum of dq/du is multiplied by the maximum
du/dt in that interval. Bracket the inverse quintic rather than using the
global 1.875/duration maximum even at the slow ends of the motion.
"""
    knots, coefficients = path_arrays(path)
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("Invalid joint path duration")
    low = np.full(coefficients.shape[2], np.inf)
    high = -low.copy()
    time_low, time_high = np.zeros_like(knots), np.ones_like(knots)
    for _ in range(52):
        mid = (time_low + time_high) / 2
        # Evaluate the symmetric half to avoid cancellation near t=1.
        half = np.minimum(mid, 1 - mid)
        progress = half**3 * (10 + half * (-15 + 6 * half))
        progress = np.where(mid <= .5, progress, 1 - progress)
        error = 32 * np.finfo(float).eps
        below = progress + error < knots
        above = progress - error > knots
        time_low = np.where(below, mid, time_low)
        time_high = np.where(above, mid, time_high)
    # Uncertain comparisons retain their bracket. These inverse values are exact.
    for progress, time in ((0, 0), (.5, .5), (1, 1)):
        time_low[knots == progress] = time
        time_high[knots == progress] = time
    speed = 0.0
    for i, width in enumerate(np.diff(knots)):
        interval_derivative = 0.0
        for j, (a, b, c, d) in enumerate(coefficients[:, i].T):
            roots = np.roots([3 * a, 2 * b, c])
            xs = [0, width] + [float(r.real) for r in roots
                               if abs(r.imag) < 1e-12 and 0 < r.real < width]
            values = [((a * x + b) * x + c) * x + d for x in xs]
            low[j], high[j] = min(low[j], *values), max(high[j], *values)
            xs = [0, width]
            if a != 0 and 0 < -b / (3 * a) < width:
                xs.append(-b / (3 * a))
            interval_derivative = max(
                interval_derivative, *(abs((3 * a * x + 2 * b) * x + c) for x in xs))
        peak_time = np.clip(.5, time_low[i], time_high[i + 1])
        envelope_speed = 30 * peak_time**2 * (1 - peak_time)**2 / duration
        speed = max(speed, interval_derivative * envelope_speed)
    return low, high, float(np.nextafter(speed, np.inf))


def validate_joint_path(path, initial, target, duration):
    knots, coefficients = path_arrays(path)
    if coefficients.shape[2] != len(initial):
        raise ValueError("Joint path has the wrong number of joints")
    # Reject serialized paths with position, velocity or acceleration jumps.
    for i, width in enumerate(np.diff(knots)[:-1]):
        a, b, c, d = coefficients[:, i]
        _, bn, cn, dn = coefficients[:, i + 1]
        values = [((a * width + b) * width + c) * width + d,
                  (3 * a * width + 2 * b) * width + c, 6 * a * width + 2 * b]
        for left, right in zip(values, (dn, cn, 2 * bn)):
            if not np.allclose(left, right, atol=1e-8, rtol=1e-8):
                raise ValueError("Joint path is not C2 continuous")
    for u, q in ((0, initial), (1, target)):
        if not np.allclose(evaluate_joint_path(path, u), q, atol=1e-8, rtol=0):
            raise ValueError("Joint path does not match command endpoints")
    return joint_path_bounds(path, duration)
