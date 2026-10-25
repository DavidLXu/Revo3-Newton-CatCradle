"""Refine a joint route while preserving all three distal thumb link poses.

This is robot-only IK. It reads a frozen rope for clearance checks and never
modifies rope positions, rest lengths, velocities, or contact constraints.
The output still requires continuous Newton replay and support tracking.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.bimanual_checks import digit_clearances, gaps
from revo3_cat_cradle.transfers import track_support_patch
from scripts.plan_bridge_transfer import free_bridge_span
from scripts.control_old_loop import nearest_helper_surface, surface_position


def preserve_thumb_pose(
    robot,
    reference,
    desired,
    obstacles,
    evaluations=100,
    helper_goal=None,
    helper_curl=None,
    fixed_wrist=False,
):
    """Return joint commands; freeze thumb PIP/DIP relative to the reference."""
    reference, desired = np.asarray(reference), np.asarray(desired)
    active = robot.arm_indices["right"] + [
        robot.indices[f"right_thumb_{joint}_joint"] for joint in ("CMP", "CMR", "MCP")
    ]
    if fixed_wrist:
        if helper_goal is None:
            raise ValueError("Fixed wrist requires a helper surface target")
        active = []
    if helper_goal is not None:
        active += [
            i for i, name in enumerate(robot.names) if name.startswith("right_middle_")
        ]
    fixed = [robot.indices[f"right_thumb_{joint}_joint"] for joint in ("PIP", "DIP")]
    trial_base = reference.copy() if fixed_wrist else desired.copy()
    trial_base[fixed] = reference[fixed]
    origin = robot.fk(reference)["right_thumb_DIP_link"]

    def residual(x):
        q = trial_base.copy()
        q[active] = x
        tf = robot.fk(q)
        tip = tf["right_thumb_DIP_link"]
        result = [
            20 * (tip[:3, 3] - origin[:3, 3]),
            0.4 * Rotation.from_matrix(tip[:3, :3] @ origin[:3, :3].T).as_rotvec(),
            0.001 * (x - desired[active]),
            30 * np.maximum(0.0015 - gaps(tf, obstacles, "right", "middle"), 0),
        ]
        if helper_goal is not None:
            link, local, world = helper_goal
            result.append(10 * ((tf[link] @ np.r_[local, 1])[:3] - world))
            if helper_curl is not None:
                result.append(
                    np.array(
                        [
                            0.1
                            * (q[robot.indices["right_middle_DIP_joint"]] - helper_curl)
                        ]
                    )
                )
        for finger in ("thumb", "index", "middle", "ring", "little"):
            result.append(
                30 * np.maximum(0.001 - digit_clearances(tf, "right", finger), 0)
            )
        return np.concatenate(result)

    lower, upper = robot.limits[active, 0], robot.limits[active, 1]
    fit = least_squares(
        residual,
        np.clip(desired[active], lower + 1e-8, upper - 1e-8),
        bounds=(lower, upper),
        max_nfev=evaluations,
    )
    trial_base[active] = fit.x
    return trial_base


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--candidates", type=Path)
    mode.add_argument(
        "--helper-lift",
        type=float,
        nargs=3,
        help="Displace a helper surface point, in world meters",
    )
    parser.add_argument("--candidate", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=12)
    parser.add_argument("--duration", type=float, default=5)
    parser.add_argument(
        "--helper-curl",
        type=float,
        help="Middle DIP angle at end of a helper move, in radians",
    )
    parser.add_argument(
        "--fixed-wrist",
        action="store_true",
        help="Move only middle joints; retain all original supporting digits exactly",
    )
    args = parser.parse_args()
    if args.steps < 1 or not np.isfinite(args.duration) or args.duration <= 0:
        parser.error("Positive steps and finite positive duration required")
    robot = RevoTron()
    if args.fixed_wrist and args.helper_lift is None:
        parser.error("--fixed-wrist requires --helper-lift")
    if args.helper_curl is not None:
        limits = robot.limits[robot.indices["right_middle_DIP_joint"]]
        if (
            args.helper_lift is None
            or not np.isfinite(args.helper_curl)
            or not limits[0] <= args.helper_curl <= limits[1]
        ):
            parser.error("Helper curl requires helper-lift and a valid DIP angle")
    state = np.load(args.source / "checkpoint.npz")
    initial = state["joints"].copy()
    target = (
        initial
        if args.candidates is None
        else np.array(
            json.loads(args.candidates.read_text())["candidates"][args.candidate][
                "joints"
            ]
        )
    )
    plan = json.loads((args.source / "transfer_plan.json").read_text())["left"]
    obstacles = state["particle_q"][
        np.union1d(plan["far_indices"], plan["near_indices"])
    ]
    helper = None
    if args.helper_lift is not None:
        if not np.isfinite(args.helper_lift).all():
            parser.error("Helper displacement must be finite")
        tf, rope = robot.fk(initial), state["particle_q"]
        patches = {}
        for side, center in [
            ("right", plan["near_indices"][-1]),
            ("left", int(np.median(plan["old_thumb_indices"]))),
        ]:
            _, patches[side], _ = track_support_patch(
                np.flatnonzero(gaps(tf, rope, side, "thumb") < 0.0015),
                len(rope),
                center,
            )
        bridge = free_bridge_span(patches["right"], patches["left"], len(rope))
        material = int(bridge[gaps(tf, rope[bridge], "right", "middle").argmin()])
        link, local = nearest_helper_surface(tf, rope[material])
        helper = (link, local, surface_position(robot, initial, link, local))
        print("Lifting robot surface next to bridge material", material)
    poses = [initial]
    for u in np.linspace(0, 1, args.steps + 1)[1:]:
        poses.append(
            preserve_thumb_pose(
                robot,
                initial,
                poses[-1] if helper else initial + u * (target - initial),
                obstacles,
                helper_goal=None
                if helper is None
                else (helper[0], helper[1], helper[2] + u * np.array(args.helper_lift)),
                helper_curl=None
                if args.helper_curl is None
                else (
                    initial[robot.indices["right_middle_DIP_joint"]] * (1 - u)
                    + u * args.helper_curl
                ),
                fixed_wrist=args.fixed_wrist,
            )
        )
    initial_tf = robot.fk(initial)
    errors, rotations, clearances, rope_gaps = [], [], [], []
    for a, b in zip(poses, poses[1:]):
        for u in np.linspace(0, 1, 9):
            tf = robot.fk(a + u * (b - a))
            for segment in ("MCP", "PIP", "DIP"):
                link = f"right_thumb_{segment}_link"
                errors.append(
                    float(np.linalg.norm(tf[link][:3, 3] - initial_tf[link][:3, 3]))
                )
                rotations.append(
                    float(
                        np.linalg.norm(
                            Rotation.from_matrix(
                                tf[link][:3, :3] @ initial_tf[link][:3, :3].T
                            ).as_rotvec()
                        )
                    )
                )
            clearances.extend(
                float(digit_clearances(tf, "right", f).min())
                for f in ("thumb", "index", "middle", "ring", "little")
            )
            rope_gaps.append(float(gaps(tf, obstacles, "right", "middle").min()))
    diagnostics = dict(
        max_thumb_link_drift_m=max(errors),
        max_thumb_link_rotation_rad=max(rotations),
        min_finger_gap_m=min(clearances),
        min_new_strand_gap_m=min(rope_gaps),
        scope="Geometric path only",
        fixed_wrist=args.fixed_wrist,
    )
    if helper is not None:
        diagnostics.update(
            helper_link=helper[0],
            helper_material=material,
            requested_displacement_m=args.helper_lift,
            helper_dip_rad=float(poses[-1][robot.indices["right_middle_DIP_joint"]]),
            actual_displacement_m=(
                surface_position(robot, poses[-1], helper[0], helper[1]) - helper[2]
            ).tolist(),
        )
        diagnostics["helper_target_error_m"] = float(
            np.linalg.norm(
                np.array(diagnostics["actual_displacement_m"]) - args.helper_lift
            )
        )
        diagnostics["helper_curl_error_rad"] = (
            0.0
            if args.helper_curl is None
            else abs(diagnostics["helper_dip_rad"] - args.helper_curl)
        )
    print(json.dumps(diagnostics, indent=2))
    # Do not emit an executable route if the sampled path fails its own checks.
    if (
        max(errors) > 0.0002
        or max(rotations) > 0.002
        or min(clearances) < 0.0005
        or min(rope_gaps) < 0.001
        or diagnostics.get("helper_target_error_m", 0) > 0.0005
        or diagnostics.get("helper_curl_error_rad", 0) > 0.05
    ):
        raise ValueError("Thumb-preserving path failed geometric checks")
    fps = json.loads(str(state["config"]))["fps"]
    frame_edges = np.rint(np.linspace(0, args.duration * fps, args.steps + 1)).astype(
        int
    )
    if np.any(np.diff(frame_edges) <= 0):
        raise ValueError("Each waypoint needs at least one output frame")
    args.output.write_text(
        json.dumps(
            [
                dict(
                    joints=q.tolist(),
                    duration=float(frame_edges[i + 1] - frame_edges[i]) / fps,
                    phase=(
                        "Lift near-thumb bridge with distal thumb pose preserved"
                        if helper
                        else "Approach bridge with distal thumb shape and pose preserved"
                    ),
                )
                for i, q in enumerate(poses[1:])
            ],
            indent=2,
        )
    )
    args.output.with_suffix(".diagnostics.json").write_text(
        json.dumps(diagnostics, indent=2)
    )


if __name__ == "__main__":
    main()
