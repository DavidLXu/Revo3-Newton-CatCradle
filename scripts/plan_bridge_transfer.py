"""Plan a wrist-and-finger approach to the original near-thumb bridge.

Only robot joints are optimized. Saved poses are geometric proposals and must
be prepared and replayed with the unchanged Newton state before use.
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
from revo3_cat_cradle.bimanual_checks import gaps, digit_clearances, hand_clearance
from scripts.plan_old_loop_support import support_geometry
from scripts.control_old_loop import nearest_helper_surface, surface_position
from revo3_cat_cradle.transfers import directed_support_arc


def free_bridge_span(right_patch, left_patch, count):
    """Interior of the known increasing-material near-thumb bridge.

    Patches are ordered along that arc. Reject overlaps and a long opposite arc
    rather than silently selecting a return strand or a point already supported.
    """
    right_patch, left_patch = np.asarray(right_patch), np.asarray(left_patch)
    if not len(right_patch) or not len(left_patch):
        raise ValueError("Both original thumb contact patches are required")
    bridge = directed_support_arc(int(right_patch[-1]), int(left_patch[0]), count, 1)[
        1:-1
    ]
    if (
        len(bridge) < 3
        or len(bridge) > count // 2
        or np.intersect1d(bridge, np.r_[right_patch, left_patch]).size
    ):
        raise ValueError("Near-thumb bridge has no unambiguous free interior")
    return bridge


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--materials", type=int, nargs="+")
    parser.add_argument("--evaluations", type=int, default=200)
    parser.add_argument(
        "--thumb-dip-min",
        type=float,
        help="Optional lower bound on right thumb tip curl, in radians",
    )
    parser.add_argument(
        "--rigid-thumb",
        action="store_true",
        help="Keep thumb PIP/DIP angles fixed so its entire distal shape is preserved",
    )
    args = parser.parse_args()
    robot = RevoTron()
    saved = np.load(args.source / "checkpoint.npz")
    q, rope = saved["joints"].copy(), saved["particle_q"].copy()
    plan = json.loads((args.source / "transfer_plan.json").read_text())["left"]
    tf0 = robot.fk(q)
    arm = robot.arm_indices["right"]
    fingers = [
        i
        for i, name in enumerate(robot.names)
        if name.startswith(("right_middle_", "right_thumb_"))
    ]
    ids = arm + fingers
    if args.rigid_thumb:
        ids = [
            i
            for i in ids
            if robot.names[i] not in ("right_thumb_PIP_joint", "right_thumb_DIP_joint")
        ]
    lower, upper = robot.limits[ids, 0].copy(), robot.limits[ids, 1].copy()
    lower[:7] = np.maximum(lower[:7], q[arm] - 0.45)
    upper[:7] = np.minimum(upper[:7], q[arm] + 0.45)
    if args.thumb_dip_min is not None:
        if args.rigid_thumb:
            parser.error("--thumb-dip-min and --rigid-thumb are incompatible")
        dip = ids.index(robot.indices["right_thumb_DIP_joint"])
        if not np.isfinite(args.thumb_dip_min) or not (
            lower[dip] <= args.thumb_dip_min < upper[dip]
        ):
            parser.error("--thumb-dip-min must lie inside the thumb DIP limits")
        lower[dip] = args.thumb_dip_min
    wrist0 = robot.wrist(q, "right")
    support_ids = np.arange(
        plan["near_indices"][-1] - 8, plan["near_indices"][-1] + 9
    ) % len(rope)
    contact = support_ids[gaps(tf0, rope[support_ids], "right", "thumb") < 0.0015]
    if not len(contact):
        raise ValueError("Original right-thumb support is absent")
    center = int(contact[(len(contact) - 1) // 2])
    link, local = nearest_helper_surface(tf0, rope[center], finger="thumb")
    support0 = surface_position(robot, q, link, local)
    support_rotation = tf0[link][:3, :3].copy()
    old_ids = np.asarray(plan["old_loop_indices"])
    old_left = old_ids[gaps(tf0, rope[old_ids], "left", "thumb") < 0.0015]
    if not len(old_left):
        raise ValueError("Original left-thumb support is absent")
    # Use the free span between current contact patches, not their old material
    # labels. Points already on a thumb cannot be an underside helper target.
    bridge = free_bridge_span(contact, old_left, len(rope))
    materials = (
        args.materials
        or np.unique(
            bridge[np.rint(np.linspace(0.2, 0.8, 3) * (len(bridge) - 1)).astype(int)]
        ).tolist()
    )
    obstacles = rope[np.union1d(plan["far_indices"], plan["near_indices"])]
    held = {
        finger: np.flatnonzero(gaps(tf0, rope, "right", finger) < 0.0015)
        for finger in ("index", "little")
    }
    rows = []
    for material in materials:
        if material not in bridge:
            raise ValueError("Target is not on the near-thumb bridge")
        for seed_number, finger_seed in enumerate(
            ([0.3, 1.3, 0.4, 0.8], [-0.2, 1.0, 1.2, 0.2], [0.1, 0.5, 1.4, 0.8])
        ):
            initial = q.copy()
            middle = [
                robot.indices[f"right_middle_{joint}_joint"]
                for joint in ("MPR", "MCP", "PIP", "DIP")
            ]
            initial[middle] = finger_seed

            def residual(x):
                trial = q.copy()
                trial[ids] = x
                tf = robot.fk(trial)
                gap, normal, axial = support_geometry(tf, rope[material], "DIP")
                wrist = tf["right_hand_base_link"]
                surface = (tf[link] @ np.r_[local, 1])[:3]
                result = [
                    np.array(
                        [
                            10 * (gap - 0.0001),
                            0.08 * max(0.85 - normal[2], 0),
                            3 * max(abs(axial) - 0.006, 0),
                        ]
                    ),
                    (10 if args.rigid_thumb else 3) * (surface - support0),
                    (0.2 if args.rigid_thumb else 0.04)
                    * Rotation.from_matrix(
                        tf[link][:3, :3] @ support_rotation.T
                    ).as_rotvec(),
                    0.15 * (wrist[:3, 3] - wrist0[:3, 3]),
                    0.001
                    * Rotation.from_matrix(
                        wrist[:3, :3] @ wrist0[:3, :3].T
                    ).as_rotvec(),
                    0.0001 * (x - q[ids]),
                    20 * np.maximum(0.002 - gaps(tf, obstacles, "right", "middle"), 0),
                ]
                for finger in ("thumb", "index", "middle", "ring", "little"):
                    result.append(
                        20
                        * np.maximum(0.001 - digit_clearances(tf, "right", finger), 0)
                    )
                for finger, materials in held.items():
                    result.append(
                        np.array(
                            [
                                2
                                * max(
                                    float(
                                        gaps(tf, rope[materials], "right", finger).min()
                                    )
                                    - 0.008,
                                    0,
                                )
                            ]
                        )
                    )
                return np.concatenate(result)

            fit = least_squares(
                residual,
                np.clip(initial[ids], lower + 1e-8, upper - 1e-8),
                bounds=(lower, upper),
                max_nfev=args.evaluations,
            )
            target = q.copy()
            target[ids] = fit.x
            tf = robot.fk(target)
            gap, normal, axial = support_geometry(tf, rope[material], "DIP")
            row = dict(
                material=material,
                seed=seed_number,
                cost=float(fit.cost),
                evaluations=int(fit.nfev),
                gap_m=gap,
                normal=normal.tolist(),
                axial_m=axial,
                support_surface_error_m=float(
                    np.linalg.norm(
                        surface_position(robot, target, link, local) - support0
                    )
                ),
                support_rotation_error_rad=float(
                    np.linalg.norm(
                        Rotation.from_matrix(
                            tf[link][:3, :3] @ support_rotation.T
                        ).as_rotvec()
                    )
                ),
                wrist_displacement_m=(
                    robot.wrist(target, "right")[:3, 3] - wrist0[:3, 3]
                ).tolist(),
                helper_new_gap_m=float(gaps(tf, obstacles, "right", "middle").min()),
                hand_gap_m=min(
                    float(digit_clearances(tf, "right", f).min())
                    for f in ("thumb", "index", "middle", "ring", "little")
                ),
                direct_opposing_gap_m=min(
                    hand_clearance(robot.fk(q + u * (target - q)))[0]
                    for u in np.linspace(0, 1, 41)
                ),
                joints=target.tolist(),
            )
            rows.append(row)
            print({k: v for k, v in row.items() if k != "joints"}, flush=True)
            args.output.write_text(
                json.dumps(
                    dict(
                        source=str(args.source),
                        time=float(saved["time"]),
                        thumb_dip_min_rad=args.thumb_dip_min,
                        rigid_thumb=args.rigid_thumb,
                        scope="Geometric proposals only; no physical transfer certified",
                        free_bridge_material=bridge.tolist(),
                        original_thumb_contacts={
                            "right": contact.tolist(),
                            "left": old_left.tolist(),
                        },
                        candidates=rows,
                    ),
                    indent=2,
                )
            )


if __name__ == "__main__":
    main()
