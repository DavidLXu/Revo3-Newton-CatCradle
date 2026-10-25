"""Trace a new thumb bight using moving support boundaries, retaining material audits.

This diagnostic follows each original support contact continuously. It records
additional patches rather than discarding them, and does not certify topology.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.bimanual_checks import gaps
from revo3_cat_cradle.mesh_geometry import (
    load_knuckle_meshes,
    support_contact_indices,
)
from revo3_cat_cradle.transfers import (
    track_support_patch,
    directed_support_arc,
    thumb_arc_sweep,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument(
        "--track-start",
        type=float,
        help="Warm-track supports from an earlier verified frame before reporting the requested interval",
    )
    args = parser.parse_args()
    track_start = args.start if args.track_start is None else args.track_start
    if not np.isfinite([args.start, track_start]).all() or track_start > args.start:
        parser.error("Tracking must start no later than the report interval")
    data = np.load(args.source / "trajectory.npz")
    plan = json.loads((args.source / "transfer_plan.json").read_text())["left"]
    count = data["rope"].shape[1]
    centers = dict(
        index=plan["far_indices"][0],
        little=plan["far_indices"][-1],
        thumb=plan["near_indices"][-1],
    )
    directions = {
        name: 1
        if (plan[name + "_indices"][1] - plan[name + "_indices"][0]) % count == 1
        else -1
        for name in ("far", "near")
    }
    robot = RevoTron()
    payload = json.loads((args.source / "trajectory.json").read_text())
    meshes = (
        load_knuckle_meshes(robot)
        if payload.get("collision_geometry", {}).get("knuckles") == "mesh"
        else {}
    )
    rows = []
    for k in np.flatnonzero(data["time"] >= track_start - 1e-4):
        tf = robot.fk(data["joints"][k])
        rope = data["rope"][k]
        row = dict(time=float(data["time"][k]), extra_contact_patches={})
        try:
            for finger, side in (
                ("index", "left"),
                ("little", "right"),
                ("thumb", "right"),
            ):
                contacts = support_contact_indices(tf, rope, side, finger, meshes)
                centers[finger], patch, extras = track_support_patch(
                    contacts, count, centers[finger]
                )
                row["extra_contact_patches"][side + "_" + finger] = [
                    p.tolist() for p in extras
                ]
            row["support_centers"] = centers.copy()
            for name, end in (("far", "little"), ("near", "thumb")):
                ids = directed_support_arc(
                    centers["index"], centers[end], count, directions[name]
                )
                distance = gaps(tf, rope[ids], "left", "thumb")
                row[name + "_gap_m"] = float(distance.min())
                row[name + "_closest_material"] = int(ids[distance.argmin()])
                row[name + "_sweep_rad"] = max(
                    thumb_arc_sweep(tf, rope[ids], "left", segment)
                    for segment in ("PIP", "DIP")
                )
                row[name + "_fixed_material_gap_m"] = float(
                    gaps(tf, rope[plan[name + "_indices"]], "left", "thumb").min()
                )
        except ValueError as exc:
            if row["time"] < args.start - 1e-4:
                raise ValueError(
                    f"Cannot initialize report: support tracking failed at {row['time']}"
                ) from exc
            row["tracking_error"] = str(exc)
            rows.append(row)
            break
        if row["time"] >= args.start - 1e-4:
            rows.append(row)
    if not rows:
        raise ValueError("No frames in interval")
    valid = [r for r in rows if "tracking_error" not in r]
    report = dict(
        scope="Sliding-support diagnostic, not a topology or physics certificate",
        track_start=track_start,
        start=rows[0]["time"],
        end=rows[-1]["time"],
        tracking_complete=len(valid) == len(rows),
        final=rows[-1],
        samples=rows,
    )
    if valid:
        report["max_far_gap_m"] = max(r["far_gap_m"] for r in valid)
        report["min_near_gap_m"] = min(r["near_gap_m"] for r in valid)
        report["min_far_sweep_deg"] = float(
            np.rad2deg(min(r["far_sweep_rad"] for r in valid))
        )
    (args.source / "sliding_bight_audit.json").write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}, indent=2))


if __name__ == "__main__":
    main()
