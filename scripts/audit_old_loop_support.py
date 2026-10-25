"""Measure helper contact direction and old-strand motion during a transfer trial."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from scripts.plan_old_loop_support import support_geometry
from scripts.plan_bridge_transfer import free_bridge_span
from revo3_cat_cradle.bimanual_checks import gaps
from revo3_cat_cradle.transfers import track_support_patch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--material", type=int, default=174)
    parser.add_argument("--window", type=int, default=8)
    parser.add_argument("--track-bridge", action="store_true")
    args = parser.parse_args()
    data = np.load(args.source / "trajectory.npz")
    plan = json.loads((args.source / "transfer_plan.json").read_text())["left"]
    ids = np.intersect1d(
        np.arange(args.material - args.window, args.material + args.window + 1),
        plan["old_loop_indices"],
    )
    if not len(ids):
        raise ValueError("No old-loop material in selected window")
    robot = RevoTron()
    rows = []
    first_rope = None
    centers = {
        "left": int(np.median(plan["old_thumb_indices"])),
        "right": plan["near_indices"][-1],
    }
    tracking_error = None
    for k in np.flatnonzero(data["time"] >= args.start - 1e-5):
        rope = data["rope"][k]
        if first_rope is None:
            first_rope = rope.copy()
        tf = robot.fk(data["joints"][k])
        tracking = {}
        if args.track_bridge:
            patches = {}
            try:
                for side in ("left", "right"):
                    contacts = np.flatnonzero(gaps(tf, rope, side, "thumb") < 0.0015)
                    centers[side], patches[side], extras = track_support_patch(
                        contacts, len(rope), centers[side]
                    )
                    tracking[side + "_other_contact_patches"] = [
                        p.tolist() for p in extras
                    ]
                ids = free_bridge_span(patches["right"], patches["left"], len(rope))
                tracking["free_bridge_material"] = ids.tolist()
                tracking["original_thumb_centers"] = centers.copy()
            except ValueError as exc:
                tracking_error = dict(time=float(data["time"][k]), reason=str(exc))
                break
        best = None
        for material in ids:
            for segment in ("MCP", "PIP", "DIP"):
                gap, normal, axial = support_geometry(tf, rope[material], segment)
                if best is None or gap < best["gap_m"]:
                    best = dict(
                        material=int(material),
                        segment=segment,
                        gap_m=gap,
                        normal=normal.tolist(),
                        axial_m=axial,
                        material_displacement_m=(
                            rope[material] - first_rope[material]
                        ).tolist(),
                    )
        rows.append(dict(time=float(data["time"][k]), **tracking, **best))
    if not rows:
        raise ValueError("No trajectory frames in interval")
    tail = [r for r in rows if r["time"] >= rows[-1]["time"] - 1]
    report = dict(
        scope="Contact direction diagnostics, not a release certificate",
        start=rows[0]["time"],
        end=rows[-1]["time"],
        selected_material=ids.tolist(),
        tracked_bridge=args.track_bridge,
        tracking_error=tracking_error,
        final_second_max_gap_m=max(r["gap_m"] for r in tail),
        final_second_min_up_normal=min(r["normal"][2] for r in tail),
        final=rows[-1],
        samples=rows,
    )
    (
        args.source
        / (
            "near_bridge_support_audit.json"
            if args.track_bridge
            else "old_loop_support_audit.json"
        )
    ).write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}, indent=2))


if __name__ == "__main__":
    main()
