"""Find a free thumb sweep to the far index string from a saved dynamic state."""

import argparse, json, sys
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.bimanual_checks import gaps
from revo3_cat_cradle.transfers import support_arc, fingertip, plan_digit_path
from revo3_cat_cradle.mesh_geometry import load_knuckle_meshes


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, default=ROOT / "output/publish_opening_a")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--side", choices=["left", "right"], default="left")
    p.add_argument("--over-height", type=float, default=0.015)
    p.add_argument(
        "--forward-offset",
        type=float,
        default=-0.012,
        help="World-X fingertip offset from the observed far strand; positive reaches beyond it",
    )
    p.add_argument(
        "--over-near",
        action="store_true",
        help="Constrain the thumb tip to pass above the measured near strand, instead of sweeping beneath it",
    )
    p.add_argument(
        "--target-index", type=int, help="Material point on the intended far strand"
    )
    p.add_argument(
        "--min-tip-height",
        type=float,
        help="Keep the fingertip above this world height throughout the sweep",
    )
    p.add_argument(
        "--support-reference",
        type=Path,
        help="Opening A checkpoint directory for material strand identities",
    )
    a = p.parse_args()
    if not np.isfinite([a.over_height, a.forward_offset]).all():
        p.error("Fingertip offsets must be finite")
    r = RevoTron()
    payload = json.loads((a.source / "trajectory.json").read_text())
    meshes = (
        load_knuckle_meshes(r)
        if payload.get("collision_geometry", {}).get("knuckles") == "mesh"
        else None
    )
    d = np.load(a.source / "checkpoint.npz")
    q = d["joints"]
    rope = d["particle_q"]
    side = a.side
    other = "right" if side == "left" else "left"
    sign = 1 if side == "left" else -1
    reference = (
        np.load(a.support_reference / "checkpoint.npz") if a.support_reference else d
    )
    far = support_arc(
        r,
        reference["joints"],
        reference["particle_q"],
        (side, "index"),
        (other, "little"),
        meshes=meshes,
    )
    near = support_arc(
        r,
        reference["joints"],
        reference["particle_q"],
        (side, "index"),
        (other, "thumb"),
        meshes=meshes,
    )
    k = (
        int(far[np.argmin(abs(rope[far, 1] - sign * 0.025))])
        if a.target_index is None
        else a.target_index
    )
    if k not in far:
        raise ValueError("Target material point must belong to the far strand")
    target = rope[k] + [a.forward_offset, 0, a.over_height]
    obstacles = rope[near]
    ids = [i for i, n in enumerate(r.names) if n.startswith(f"{side}_thumb_")]

    def residual(x):
        trial = q.copy()
        trial[ids] = x
        return np.r_[
            (fingertip(r, trial, side, "thumb") - target) * 100,
            np.minimum(gaps(r.fk(trial), obstacles, side, "thumb") - 0.003, 0) * 150,
            0.01 * (x - q[ids]),
        ]

    fit = least_squares(
        residual, q[ids], bounds=(r.limits[ids, 0], r.limits[ids, 1]), max_nfev=300
    )
    goal = q.copy()
    goal[ids] = fit.x
    error = np.linalg.norm(fingertip(r, goal, side, "thumb") - target)
    if error > 0.003:
        raise ValueError(
            f"Thumb approach target is unreachable: residual {error * 1000:.1f} mm at {target.tolist()}"
        )
    route = plan_digit_path(
        r,
        q,
        goal,
        side,
        "thumb",
        obstacles,
        min_tip_height=a.min_tip_height,
        over_points=obstacles if a.over_near else None,
    )
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps([v.tolist() for v in route], indent=2))
    print(
        f"Saved {len(route)} approach waypoints to {a.output}. Must still be verified in contact simulation."
    )


if __name__ == "__main__":
    main()
