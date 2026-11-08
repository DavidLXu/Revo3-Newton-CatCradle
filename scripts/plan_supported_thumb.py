"""Propose one left wrist/thumb/index step from the observed rope state.

Only joint commands are produced. Frozen rope geometry supplies planning costs;
it cannot establish retention, threading or a completed figure. Replay the route
in Newton with continuation guards and audit the original material tracks.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.bimanual_checks import (
    gaps, digit_clearances, self_clearance_margin, hand_clearance,
)
from revo3_cat_cradle.transfers import fingertip
from revo3_cat_cradle.palm_clearance import OpposingPalmClearance
from scripts.control_old_loop import nearest_helper_surface


def validate_observation(time, duration, tracking, metrics, *, stop_reports=(), near_gaps=None):
    """Do not propose a continuation from stale or already rejected evidence."""
    if stop_reports:
        raise ValueError("Source contains a failed continuation stop report")
    if near_gaps is not None and (
        not np.isfinite(near_gaps).all() or np.min(near_gaps) < 0.0015
    ):
        raise ValueError("Source already contacts a forbidden near strand")
    if not np.isfinite([time, duration]).all() or abs(time - duration) > 1e-5:
        raise ValueError("Source motion did not finish at its saved checkpoint")
    final = tracking.get("final", {})
    if not np.isclose(final.get("time", np.nan), time, atol=1e-5, rtol=0):
        raise ValueError("Material tracking does not describe the saved checkpoint")
    if not tracking.get("tracking_complete") or tracking.get("errors"):
        raise ValueError("Original support tracking failed")
    supports = final.get("supports", {})
    for side in ("left", "right"):
        for finger in ("thumb", "index"):
            if not supports.get(f"{side}_{finger}", {}).get("patch"):
                raise ValueError("An original support patch is missing")
    strain = np.array([row["max_stretch_ratio"] for row in metrics])
    if not len(strain) or not np.isfinite(strain).all() or strain.max() >= 1.2:
        raise ValueError("Source history fails the unchanged strain gate")


def propose(robot, q, rope, plan, supports, *, particle, offset, duration,
            speed=1.5, retention_weight=500, index_weight=100):
    """Optimize a robot-only step; contact costs are soft planning objectives."""
    q, rope = np.array(q, copy=True), np.asarray(rope)
    tf0 = robot.fk(q)
    old = rope[supports["left_thumb"]["patch"]]
    near = rope[plan["left"]["near_indices"]]
    link, local = nearest_helper_surface(
        tf0, rope[supports["left_index"]["center"]], "left", "index"
    )
    reference = (tf0[link] @ np.r_[local, 1])[:3]
    target = rope[particle] + np.asarray(offset)
    ids = robot.arm_indices["left"] + [
        i for i, name in enumerate(robot.names)
        if name.startswith(("left_thumb_", "left_index_"))
    ]
    # The continuation tool uses a quintic with peak slope 1.875.
    bound = speed * duration / 1.875

    def residual(x):
        trial = q.copy()
        trial[ids] = x
        tf = robot.fk(trial)
        result = [
            20 * (fingertip(robot, trial, "left", "thumb") - target),
            index_weight * ((tf[link] @ np.r_[local, 1])[:3] - reference),
            0.001 * (x - q[ids]),
        ]
        for u in (0.25, 0.5, 0.75, 1):
            poses = robot.fk(q + u * (trial - q))
            old_gap = gaps(poses, old, "left", "thumb")
            result.extend([
                retention_weight * np.maximum(-0.0002 - old_gap, 0),
                np.array([retention_weight * max(old_gap.min() - 0.0001, 0)]),
                50 * np.maximum(0.003 - gaps(poses, near, "left", "thumb"), 0),
            ])
            for finger in ("thumb", "index"):
                result.append(
                    30 * np.maximum(0.0005 - digit_clearances(poses, "left", finger), 0)
                )
        return np.concatenate(result)

    fit = least_squares(
        residual, q[ids],
        bounds=(np.maximum(robot.limits[ids, 0], q[ids] - bound),
                np.minimum(robot.limits[ids, 1], q[ids] + bound)),
        max_nfev=160, ftol=1e-8, xtol=1e-8, gtol=1e-8,
    )
    goal = q.copy()
    goal[ids] = fit.x
    return goal, dict(
        particle=particle, target=target.tolist(),
        initial_error_m=float(np.linalg.norm(fingertip(robot, q, "left", "thumb") - target)),
        final_error_m=float(np.linalg.norm(fingertip(robot, goal, "left", "thumb") - target)),
        evaluations=int(fit.nfev),
    ), (link, local, reference)


def preflight(robot, initial, goal, rope, plan, supports, reference, duration):
    palms = OpposingPalmClearance(robot, robot.fk(initial))
    link, local, position = reference
    rows = []
    for u in np.linspace(0, 1, 21):
        tf = robot.fk(initial + u * (goal - initial))
        palm = palms.measure(tf, quick=True)
        rows.append(dict(
            fraction=float(u),
            near_gap_m={side: float(gaps(tf, rope[plan[side]["near_indices"]], side, "thumb").min())
                        for side in ("left", "right")},
            old_thumb_gap_m=float(gaps(tf, rope[supports["left_thumb"]["patch"]], "left", "thumb").min()),
            index_reference_drift_m=float(np.linalg.norm((tf[link] @ np.r_[local, 1])[:3] - position)),
            self_margin_m=min(self_clearance_margin(tf, s) for s in ("left", "right")),
            opposing_gap_m=hand_clearance(tf)[0],
            finger_palm_gap_m=palm["finger_palm_lower_bound_m"],
            palm_sphere_gap_m=palm["palm_palm_sphere_lower_bound_m"],
        ))
    return rows, float(1.875 * np.max(abs(goal - initial)) / duration)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--particle", type=int, default=170)
    parser.add_argument("--offset", type=float, nargs=3, default=[0.005, 0, -0.005])
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--speed", type=float, default=1.5)
    parser.add_argument("--retention-weight", type=float, default=500)
    parser.add_argument("--index-weight", type=float, default=100)
    parser.add_argument("--right-thumb-mcp", type=float,
                        help="Optional simultaneous right-thumb joint target, radians")
    args = parser.parse_args()
    values = [args.duration, args.speed, args.retention_weight, args.index_weight]
    if not np.isfinite(values + args.offset).all() or min(values) <= 0:
        parser.error("Finite positive duration, speed and weights are required")
    if (round(args.duration * 30) < 1
            or abs(args.duration * 30 - round(args.duration * 30)) > 1e-5):
        parser.error("Duration must lie on the continuation tool's 30 Hz frame grid")
    if args.output.exists():
        parser.error("Use a new output path to preserve previous proposals")
    with np.load(args.source / "checkpoint.npz") as saved:
        q, rope, time = saved["joints"].copy(), saved["particle_q"].copy(), float(saved["time"])
    payload = json.loads((args.source / "trajectory.json").read_text())
    tracking = json.loads((args.source / "material_support_tracking.json").read_text())
    metrics = json.loads((args.source / "metrics.json").read_text())["metrics"]
    if not np.isfinite(q).all() or not np.isfinite(rope).all():
        raise ValueError("Nonfinite source geometry")
    plan = json.loads((args.source / "transfer_plan.json").read_text())
    robot = RevoTron()
    tf = robot.fk(q)
    validate_observation(
        time, payload["motion"]["duration"], tracking, metrics,
        stop_reports=list(args.source.glob("*_stop.json")),
        near_gaps=[float(gaps(tf, rope[plan[side]["near_indices"]], side, "thumb").min())
                   for side in ("left", "right")],
    )
    if args.particle not in plan["left"]["far_indices"]:
        raise ValueError("Target particle must belong to the intended free strand")
    supports = tracking["final"]["supports"]
    goal, report, reference = propose(
        robot, q, rope, plan, supports, particle=args.particle, offset=args.offset,
        duration=args.duration, speed=args.speed, retention_weight=args.retention_weight,
        index_weight=args.index_weight,
    )
    if args.right_thumb_mcp is not None:
        goal[robot.indices["right_thumb_MCP_joint"]] = args.right_thumb_mcp
    if (not np.isfinite(goal).all() or np.any(goal < robot.limits[:, 0])
            or np.any(goal > robot.limits[:, 1])):
        raise ValueError("Proposed joints violate their limits")
    samples, peak_speed = preflight(robot, q, goal, rope, plan, supports, reference, args.duration)
    passed = peak_speed <= args.speed + 1e-7 and all(
        row["near_gap_m"]["left"] >= 0.0015 and row["self_margin_m"] >= 0
        and row["opposing_gap_m"] >= -0.0005 and row["finger_palm_gap_m"] >= 0
        and row["palm_sphere_gap_m"] >= 0 for row in samples
    )
    report.update(
        source=str(args.source), source_time_s=time, duration_s=args.duration,
        retention_weight=args.retention_weight, index_weight=args.index_weight,
        right_thumb_mcp=args.right_thumb_mcp, peak_joint_speed_rad_s=peak_speed,
        sampled_geometry_passed=passed, samples=samples,
        frozen_near_guard_sides=["left"],
        scope="Robot-only proposal. Old contact and index reference are soft objectives; "
              "frozen-rope distances cannot certify retention or a captured bight. "
              "The optional right-thumb relief is measured but not gated against "
              "frozen near material; guard both actual near strands during Newton replay.",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".preflight.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}, indent=2))
    if not passed:
        raise ValueError("Sampled joint path rejected; no executable route emitted")
    args.output.write_text(json.dumps([dict(
        duration=args.duration, joints=goal.tolist(),
        phase="Coordinate the left wrist, thumb and index toward the free strand",
    )], indent=2) + "\n")


if __name__ == "__main__":
    main()
