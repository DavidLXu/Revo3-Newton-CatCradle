"""Find joint-only helper poses below a measured old thumb strand.

Static contact geometry is a planning hint. Every route requires a continuous
Newton replay; a reachable endpoint alone does not establish a transferred loop.
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
from revo3_cat_cradle.bimanual_checks import digit_clearances, gaps
from revo3_cat_cradle.contact_geometry import finger_capsule


def support_geometry(
    transforms, point, segment="PIP", rope_radius=0.001, side="right", finger="middle"
):
    """Gap, world outward normal and axial coordinate at a helper capsule."""
    center, axis, radius, half = finger_capsule(finger, segment, side)
    tf = transforms[f"{side}_{finger}_{segment}_link"]
    local = (point - tf[:3, 3]) @ tf[:3, :3]
    axial = (local - center) @ axis
    delta = local - center - np.clip(axial, -half, half) * axis
    distance = np.linalg.norm(delta)
    normal = tf[:3, :3] @ (delta / max(distance, 1e-12))
    return float(distance - radius - rope_radius), normal, float(axial)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--materials", type=int, nargs="+", default=[170, 174, 178])
    parser.add_argument("--segment", choices=["PIP", "DIP"], default="PIP")
    parser.add_argument("--side", choices=["left", "right"], default="right")
    parser.add_argument("--finger", choices=["middle", "ring"], default="middle")
    parser.add_argument("--wrist-offset", type=float, nargs=3, default=[0, 0, 0])
    args = parser.parse_args()
    saved = np.load(args.source / "checkpoint.npz")
    plan = json.loads((args.source / "transfer_plan.json").read_text())["left"]
    robot = RevoTron()
    q, rope = saved["joints"].copy(), saved["particle_q"].copy()
    if np.any(args.wrist_offset):
        wrist = robot.wrist(q, args.side)
        q = robot.ik(q, args.side, wrist[:3, 3] + args.wrist_offset, wrist[:3, :3])
    obstacles = rope[np.union1d(plan["far_indices"], plan["near_indices"])]
    ids = [
        i
        for i, name in enumerate(robot.names)
        if name.startswith(f"{args.side}_{args.finger}_")
    ]
    rows = []
    for material in args.materials:
        if material not in plan["old_loop_indices"]:
            raise ValueError("Requested material is outside the old thumb loop")
        for seed in ([0.3, 0.7, 0.7, 0.8], [0.3, 1.1, 0.5, 0.8], [0.3, 1.5, 0.3, 0.8]):

            def residual(x):
                trial = q.copy()
                trial[ids] = x
                tf = robot.fk(trial)
                gap, normal, axial = support_geometry(
                    tf, rope[material], args.segment, side=args.side, finger=args.finger
                )
                return np.r_[
                    10 * (gap - 0.0001),
                    0.05 * max(0.85 - normal[2], 0),
                    3
                    * max(abs(axial) - (0.009 if args.segment == "PIP" else 0.006), 0),
                    0.003 * (x[-1] - 0.8),
                    20
                    * np.maximum(
                        0.0015 - gaps(tf, obstacles, args.side, args.finger), 0
                    ),
                    20
                    * np.maximum(
                        0.001 - digit_clearances(tf, args.side, args.finger), 0
                    ),
                ]

            fit = least_squares(
                residual,
                seed,
                bounds=(robot.limits[ids, 0], robot.limits[ids, 1]),
                max_nfev=160,
            )
            trial = q.copy()
            trial[ids] = fit.x
            tf = robot.fk(trial)
            gap, normal, axial = support_geometry(
                tf, rope[material], args.segment, side=args.side, finger=args.finger
            )
            path_new, path_hand = np.inf, np.inf
            for u in np.linspace(0, 1, 41):
                step = robot.fk(q + u * (trial - q))
                path_new = min(
                    path_new, float(gaps(step, obstacles, args.side, args.finger).min())
                )
                path_hand = min(
                    path_hand,
                    float(digit_clearances(step, args.side, args.finger).min()),
                )
            row = dict(
                material=material,
                segment=args.segment,
                side=args.side,
                finger=args.finger,
                gap_m=gap,
                normal=normal.tolist(),
                axial_m=axial,
                new_gap_m=float(gaps(tf, obstacles, args.side, args.finger).min()),
                hand_gap_m=float(digit_clearances(tf, args.side, args.finger).min()),
                direct_path_new_gap_m=path_new,
                direct_path_hand_gap_m=path_hand,
                joints=trial.tolist(),
            )
            rows.append(row)
            print({k: v for k, v in row.items() if k != "joints"}, flush=True)
    args.output.write_text(
        json.dumps(
            dict(
                source=str(args.source),
                time=float(saved["time"]),
                wrist_offset_m=args.wrist_offset,
                preparation_joints=q.tolist(),
                scope="Static planning only; wrist displacement requires physical preparation and replanning",
                candidates=rows,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
