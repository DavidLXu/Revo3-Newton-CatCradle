"""Inspect pickup/release timing without certifying a completed string figure."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.bimanual_checks import gaps, segment_separation
from revo3_cat_cradle.transfers import thumb_arc_sweep
from revo3_cat_cradle.mesh_geometry import load_knuckle_meshes, support_gap


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--side", choices=["left", "right"], default="left")
    p.add_argument("--start", type=float, required=True)
    p.add_argument(
        "--output",
        type=Path,
        help="Optional report path for separate left/right audits",
    )
    a = p.parse_args()
    data = np.load(a.source / "trajectory.npz")
    payload = json.loads((a.source / "trajectory.json").read_text())
    plan = json.loads((a.source / "transfer_plan.json").read_text())[a.side]
    robot = RevoTron()
    meshes = (
        load_knuckle_meshes(robot)
        if payload.get("collision_geometry", {}).get("knuckles") == "mesh"
        else {}
    )
    other = "right" if a.side == "left" else "left"
    rows = []
    for k in np.flatnonzero(data["time"] >= a.start):
        rope = data["rope"][k]
        tf = robot.fk(data["joints"][k])
        row = dict(time=float(data["time"][k]))
        for key in ("far_indices", "near_indices", "old_loop_indices"):
            points = rope[plan[key]]
            row[key + "_gap_m"] = float(gaps(tf, points, a.side, "thumb").min())
        for key in ("far_indices", "near_indices"):
            row[key + "_sweep_rad"] = max(
                thumb_arc_sweep(tf, rope[plan[key]], a.side, segment)
                for segment in ("PIP", "DIP")
            )
        row["donor_little_gap_m"] = support_gap(tf, rope, other, "little", meshes)
        row["other_support_max_gap_m"] = max(
            support_gap(tf, rope, side, finger, meshes)
            for side, finger in (
                (a.side, "index"),
                (other, "index"),
                (a.side, "little"),
            )
        )
        lengths = np.linalg.norm(np.roll(rope, -1, axis=0) - rope, axis=1)
        row["stretch"] = float((lengths / data["rest_lengths"][k]).max())
        row["nonlocal_separation_m"] = segment_separation(
            rope, minimum_arc_distance=2 * payload["config"]["radius"]
        )
        rows.append(row)
    if not rows:
        raise ValueError("No frames in the requested interval")

    def first(predicate):
        return next((r["time"] for r in rows if predicate(r)), None)

    last = [r for r in rows if r["time"] >= rows[-1]["time"] - 1]
    report = dict(
        scope="Timing diagnostics only; angular sweep does not certify topology",
        side=a.side,
        start_s=rows[0]["time"],
        end_s=rows[-1]["time"],
        support_geometry=payload.get("collision_geometry", {}),
        first_far_contact_s=first(lambda r: r["far_indices_gap_m"] < 0.0015),
        first_donor_little_loss_s=first(lambda r: r["donor_little_gap_m"] > 0.0015),
        first_far_contact_and_half_turn_s=first(
            lambda r: r["far_indices_gap_m"] < 0.0015
            and r["far_indices_sweep_rad"] >= np.pi
        ),
        final_hold_max_far_gap_m=max(r["far_indices_gap_m"] for r in last),
        final_hold_min_far_sweep_deg=float(
            np.rad2deg(min(r["far_indices_sweep_rad"] for r in last))
        ),
        max_other_support_gap_m=max(r["other_support_max_gap_m"] for r in rows),
        max_old_loop_gap_m=max(r["old_loop_indices_gap_m"] for r in rows),
        peak_stretch=max(r["stretch"] for r in rows),
        minimum_nonlocal_separation_m=min(r["nonlocal_separation_m"] for r in rows),
        samples=rows,
    )
    (a.output or a.source / "pickup_timing_audit.json").write_text(
        json.dumps(report, indent=2)
    )
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}, indent=2))


if __name__ == "__main__":
    main()
