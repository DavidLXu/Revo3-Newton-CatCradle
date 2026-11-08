"""Fixed-material free-loop measurements and a turning-point timing cue.

These signals guide joint motion; they neither move the rope nor certify a
crossing, a retained loop, or a completed figure.
"""

from dataclasses import dataclass

import numpy as np


def validate_bilateral_release(report, checkpoint_time):
    """Reject retained/one-sided little-loop phases before a free-loop swing."""
    interval = report.get("interval_s", [])
    checks = report.get("checks", {})
    if (not report.get("passed_diagnostics")
            or set(report.get("released_sides", [])) != {"left", "right"}
            or not all(checks.get(side + "_little_clear") for side in ("left", "right"))
            or not interval or not np.isfinite([interval[-1], checkpoint_time]).all()
            or abs(interval[-1] - checkpoint_time) > 1e-5):
        raise ValueError("Source needs completed bilateral little-loop release at its checkpoint")


def common_wrist_translation(robot, initial, targets, duration, steps):
    """Approximate a common Cartesian translation with bounded joint segments.

    Joint interpolation alone does not preserve the relative wrist transform.
    Sample that transform within every segment and reject excessive drift.
    Each segment uses Motion's existing zero-end-speed quintic interpolation.
    """
    from scipy.spatial.transform import Rotation

    if steps < 1 or duration <= 0 or not np.isfinite(duration):
        raise ValueError("Invalid Cartesian segment timing")
    dt = duration / steps
    if abs(dt * 30 - round(dt * 30)) > 1e-7:
        raise ValueError("Cartesian segments must lie on the 30 Hz frame grid")
    wrists = {s: robot.wrist(initial, s).copy() for s in ("left", "right")}
    relative0 = np.linalg.inv(wrists["left"]) @ wrists["right"]
    q = initial.copy()
    route = []
    translation_drift = rotation_drift = speed = 0.0
    for step in range(1, steps + 1):
        before = q.copy()
        u = step / steps
        for side, pose in wrists.items():
            point = (1 - u) * pose[:3, 3] + u * np.asarray(targets[side])
            q = robot.ik(q, side, point, pose[:3, :3])
        speed = max(speed, float(np.max(abs(q - before))) * 1.875 / dt)
        for v in np.linspace(0, 1, 11):
            trial = before * (1 - v) + q * v
            relative = np.linalg.inv(robot.wrist(trial, "left")) @ robot.wrist(trial, "right")
            translation_drift = max(translation_drift, float(np.linalg.norm(relative[:3, 3] - relative0[:3, 3])))
            rotation_drift = max(rotation_drift, float(np.linalg.norm(Rotation.from_matrix(relative[:3, :3] @ relative0[:3, :3].T).as_rotvec())))
        route.append(dict(duration=dt, joints=q.tolist(), phase="Swing the free loop using observed turning points"))
    report = dict(peak_joint_speed_rad_s=speed, maximum_relative_translation_drift_m=translation_drift,
                  maximum_relative_rotation_drift_deg=float(np.rad2deg(rotation_drift)), segments=steps)
    if translation_drift > 0.001 or rotation_drift > np.deg2rad(0.2):
        raise ValueError(f"Joint path fails relative wrist tolerance: {report}")
    if speed > 3:
        raise ValueError(f"Joint path exceeds 3 rad/s: {report}")
    return route, report


def free_loop_material(plan, count):
    left = int(plan["left"]["far_indices"][0])
    right = int(plan["right"]["far_indices"][0])
    thumbs = np.array([
        int(plan[s]["old_thumb_indices"][(len(plan[s]["old_thumb_indices"]) - 1) // 2])
        for s in ("left", "right")
    ])
    if count < 17 or not all(0 <= i < count for i in [left, right, *thumbs]):
        raise ValueError("Invalid original support material")
    arcs = [
        (right + np.arange((left - right) % count + 1)) % count,
        (right - np.arange((right - left) % count + 1)) % count,
    ]
    free = [arc for arc in arcs if not np.intersect1d(arc, thumbs).size]
    if len(free) != 1 or len(free[0]) < 17:
        raise ValueError("Cannot identify the original free arc")
    middle = len(free[0]) // 2
    return free[0][middle - 8:middle + 9], thumbs


@dataclass
class ApexDetector:
    """Arm on outward motion, then detect its reversal on the intended side."""

    direction: int
    minimum_offset: float = 0.03
    minimum_speed: float = 0.03
    armed: bool = False

    def __post_init__(self):
        if self.direction not in (-1, 1) or not np.isfinite(
            [self.minimum_offset, self.minimum_speed]
        ).all() or min(self.minimum_offset, self.minimum_speed) <= 0:
            raise ValueError("Invalid turning-point detector")

    def update(self, position_x, velocity_x):
        if not np.isfinite([position_x, velocity_x]).all():
            raise ValueError("Nonfinite free-loop observation")
        x, v = self.direction * position_x, self.direction * velocity_x
        if x < self.minimum_offset:
            self.armed = False
            return False
        if v >= self.minimum_speed:
            self.armed = True
        return self.armed and v <= 0
