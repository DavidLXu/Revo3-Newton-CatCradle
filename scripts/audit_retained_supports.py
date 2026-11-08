"""Follow original material supports through clamping and little-loop release.

This is a material-continuity diagnostic, not a threading certificate. Extra
contact patches remain in the report. Particle samples and enabled knuckle
meshes are checked; rope-edge contact alone cannot establish a material track.
The optional six-support audit also follows the original little loops before
their intended release and writes a separate report.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.mesh_geometry import load_knuckle_meshes, support_contact_indices
from revo3_cat_cradle.transfers import track_support_patch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--include-little", action="store_true",
                        help="Before release, also track original little-loop material; "
                             "writes a separate six_material_support_tracking.json")
    parser.add_argument(
        "--start", type=float, required=True,
        help="Recorded reference frame whose original supports match transfer_plan.json",
    )
    args = parser.parse_args()
    with np.load(args.source / "trajectory.npz") as archive:
        data = {key: archive[key] for key in ("time", "joints", "rope")}
    times = data["time"]
    if not np.isfinite(args.start) or args.start < times[0] or args.start > times[-1]:
        parser.error("Start must be finite and inside the recorded interval")
    selected = np.flatnonzero(times >= args.start - 1e-5)
    if not len(selected) or abs(times[selected[0]] - args.start) > 1e-5:
        parser.error("Start must match a recorded frame")
    payload = json.loads((args.source / "trajectory.json").read_text())
    if abs(payload["config"]["radius"] - 0.001) > 1e-10:
        parser.error("The support-distance helper currently assumes a 1 mm rope radius")
    plan = json.loads((args.source / "transfer_plan.json").read_text())
    centers = {}
    for side in ("left", "right"):
        old = plan[side]["old_thumb_indices"]
        centers[side + "_thumb"] = int(old[(len(old) - 1) // 2])
        centers[side + "_index"] = int(plan[side]["far_indices"][0])
        if args.include_little:
            other = "right" if side == "left" else "left"
            # The far index string ends on the opposite hand's little finger.
            centers[side + "_little"] = int(plan[other]["far_indices"][-1])
    initial_centers = centers.copy()
    robot = RevoTron()
    meshes = (
        load_knuckle_meshes(robot)
        if payload.get("collision_geometry", {}).get("knuckles") == "mesh"
        else {}
    )
    errors = {}
    rows = []
    for k in selected:
        rope = data["rope"][k]
        transforms = robot.fk(data["joints"][k])
        row = dict(time=float(times[k]), supports={})
        for name in centers:
            if name in errors:
                continue
            side, finger = name.split("_")
            contacts = support_contact_indices(transforms, rope, side, finger, meshes)
            try:
                centers[name], patch, extras = track_support_patch(
                    contacts, len(rope), centers[name]
                )
                row["supports"][name] = dict(
                    center=centers[name],
                    patch=patch.tolist(),
                    extras=[part.tolist() for part in extras],
                )
            except ValueError as exc:
                errors[name] = dict(time_s=float(times[k]), reason=str(exc))
        rows.append(row)
    report = dict(
        scope=__doc__,
        interval_s=[float(times[selected[0]]), float(times[-1])],
        initial_centers=initial_centers,
        tracking_method="midpoint_with_retained_material_sample",
        tracked_supports=list(initial_centers),
        maximum_center_shift_per_frame_particles=8,
        tracking_complete=not errors,
        errors=errors,
        final=rows[-1],
        samples=rows,
    )
    filename = ("six_material_support_tracking.json" if args.include_little
                else "material_support_tracking.json")
    (args.source / filename).write_text(
        json.dumps(report, indent=2) + "\n"
    )
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}, indent=2))
    return 0 if report["tracking_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
