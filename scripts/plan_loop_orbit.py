"""Propose a common wrist orbit from a held, released-loop checkpoint.

This writes robot joint commands only. The proposal must be integrated in Newton
and audited; the geometric preflight does not establish a grip or a loop flip.
"""

import argparse
import hashlib
import json
from math import factorial
from pathlib import Path
import sys

import numpy as np
from scipy.interpolate import BSpline
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.free_loop import (
    validate_bilateral_near_hold,
    validate_bilateral_release,
)
from revo3_cat_cradle.joint_path import evaluate_joint_path, validate_joint_path


def plan(robot, initial, *, duration, radius_x, radius_z, cycles=3,
         late_x=0., late_z=0., late_start=.60, late_end=.76):
    """Fit a bounded cubic B-spline to wrist IK control points.

    A common translation preserves the intended relative wrist transform.
    Sampling checks the error introduced by the joint-space approximation.
    One global quintic time envelope is applied later by Motion.
    """
    numbers = [duration, radius_x, radius_z, late_x, late_z, late_start, late_end]
    if (not np.isfinite(numbers).all() or duration <= 0
            or min(radius_x, radius_z) <= 0
            or not isinstance(cycles, int) or cycles < 1
            or not 0 <= late_start < late_end <= 1
            or abs(duration * 30 - round(duration * 30)) > 1e-7):
        raise ValueError("Use positive radii, integer cycles, ordered phase bounds "
                         "and a positive duration on the 30 Hz frame grid")
    wrists = {s: robot.wrist(initial, s).copy() for s in ("left", "right")}
    relative0 = np.linalg.inv(wrists["left"]) @ wrists["right"]

    def delta(u):
        angle = 2 * np.pi * cycles * u
        v = np.clip((u - late_start) / (late_end - late_start), 0., 1.)
        ramp = v**3 * (10 - 15*v + 6*v*v)
        return np.array([radius_x * (1 - np.cos(angle)) + late_x*ramp,
                         0., -radius_z * np.sin(angle) + late_z*ramp])

    breaks = np.linspace(0, 1, cycles * 36 + 1)
    knots = np.r_[np.zeros(3), breaks, np.ones(3)]
    progress = [np.mean(knots[i+1:i+4]) for i in range(len(knots) - 4)]
    q, targets = initial.copy(), []
    for u in progress:
        if u:
            for side, pose in wrists.items():
                q = robot.ik(q, side, pose[:3, 3] + delta(u), pose[:3, :3])
        targets.append(q.copy())
    # IK samples are control points, not interpolation points. Their convex
    # hull avoids joint-limit overshoot; actual wrist error is checked below.
    spline = BSpline(knots, np.array(targets), 3, axis=0)
    path = dict(knots=breaks.tolist(), coefficients=np.stack([
        spline(breaks[:-1], nu=k) / factorial(k) for k in (3, 2, 1, 0)
    ]).tolist())
    low, high, speed = validate_joint_path(path, initial, q, duration)
    drift = rotation = cartesian = 0.
    for u in np.linspace(0, 1, cycles * 360 + 1):
        trial = evaluate_joint_path(path, u)
        poses = {s: robot.wrist(trial, s) for s in wrists}
        relative = np.linalg.inv(poses["left"]) @ poses["right"]
        drift = max(drift, float(np.linalg.norm(
            relative[:3, 3] - relative0[:3, 3])))
        rotation = max(rotation, float(np.rad2deg(np.linalg.norm(
            Rotation.from_matrix(relative[:3, :3] @ relative0[:3, :3].T).as_rotvec()))))
        cartesian = max(cartesian, max(float(np.linalg.norm(
            poses[s][:3, 3] - wrists[s][:3, 3] - delta(u))) for s in wrists))
    checks = dict(
        joint_limits=bool(np.all(low >= robot.limits[:, 0] - 1e-6)
                          and np.all(high <= robot.limits[:, 1] + 1e-6)),
        speed_bound=bool(speed <= 3),
        relative_translation=bool(drift < .001),
        relative_rotation=bool(rotation < .2),
        cartesian_approximation=bool(cartesian < .001),
    )
    report = dict(
        scope=__doc__, passed=all(checks.values()), checks=checks,
        cycles=cycles, duration_s=duration, radius_x_m=radius_x, radius_z_m=radius_z,
        late_translation_m=[late_x, 0., late_z],
        late_translation_phase_interval=[late_start, late_end],
        phase_envelope="One global quintic time envelope",
        peak_orbit_frequency_hz=cycles * 1.875 / duration,
        peak_joint_speed_bound_rad_s=speed,
        relative_translation_drift_m=drift, relative_rotation_deg=rotation,
        maximum_cartesian_error_m=cartesian,
    )
    route = [dict(duration=duration, joints=q.tolist(), joint_path=path,
                  phase="Common wrist orbit with optional late translation")]
    return route, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=5.)
    parser.add_argument("--radius-x", type=float, default=.08)
    parser.add_argument("--radius-z", type=float, default=.08)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--late-x", type=float, default=0.)
    parser.add_argument("--late-z", type=float, default=0.)
    parser.add_argument("--late-start", type=float, default=.60)
    parser.add_argument("--late-end", type=float, default=.76)
    args = parser.parse_args()
    report_path = args.output.with_suffix(".preflight.json")
    if args.output == report_path or args.output.exists() or report_path.exists():
        parser.error("Choose an unused .json recipe path distinct from its preflight report")
    checkpoint = args.source / "checkpoint.npz"
    with np.load(checkpoint) as state:
        initial, time = state["joints"].copy(), float(state["time"])
    validate_bilateral_release(json.loads(
        (args.source / "loop_release_audit.json").read_text()), time)
    validate_bilateral_near_hold(json.loads(
        (args.source / "near_pinch_edge_audit.json").read_text()), time)
    route, report = plan(
        RevoTron(), initial, duration=args.duration, radius_x=args.radius_x,
        radius_z=args.radius_z, cycles=args.cycles, late_x=args.late_x,
        late_z=args.late_z, late_start=args.late_start, late_end=args.late_end,
    )
    report.update(source=str(args.source), source_time_s=time,
                  source_checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("Orbit failed geometric preflight; no recipe written")
    args.output.write_text(json.dumps(route, separators=(",", ":"), allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
