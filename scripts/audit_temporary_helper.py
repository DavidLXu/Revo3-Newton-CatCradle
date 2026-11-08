"""Track declared temporary old-loop contact on an otherwise unused finger.

This measures material continuity only. A contacting rope point can slide along
the finger; the report does not certify a grasp, release, physical validity or
a completed figure. It never changes the rope or robot commands.
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
    parser.add_argument("--intent", type=Path,
                        help="Predeclared helper phase; defaults to the source intent file")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    intent = json.loads((args.intent or args.source / "helper_phase_intent.json").read_text())
    helper = intent["temporary_helper"]
    side, finger = helper["side"], helper["finger"]
    start, end = helper["expected_contact_interval_s"]
    center = helper["initial_material_center"]
    if (side not in ("left", "right") or finger not in ("middle", "ring", "little")
            or not np.isfinite([start, end]).all() or start >= end
            or not isinstance(center, int)):
        parser.error("Invalid temporary-helper declaration")
    with np.load(args.source / "trajectory.npz") as archive:
        times, joints, rope = (archive[k].copy() for k in ("time", "joints", "rope"))
    payload = json.loads((args.source / "trajectory.json").read_text())
    plan = json.loads((args.source / "transfer_plan.json").read_text())
    fps = float(payload["config"]["fps"])
    if (not np.isfinite(fps) or fps <= 0 or not len(times)
            or not all(np.isfinite(x).all() for x in (times, joints, rope))
            or np.any(np.diff(times) <= 0) or start < times[0] - 1e-5
            or abs(start * fps - round(start * fps)) > 1e-4
            or abs(end * fps - round(end * fps)) > 1e-4):
        parser.error("Invalid trajectory or phase times")
    arc = np.asarray(plan[side]["old_loop_indices"], dtype=int)
    window = np.asarray(helper["old_loop_material_window"], dtype=int)
    if (not 0 <= center < rope.shape[1] or center not in window
            or not np.isin(window, arc).all()):
        parser.error("Helper seed must belong to its declared original old-loop window")
    robot = RevoTron()
    meshes = (load_knuckle_meshes(robot)
              if payload["collision_geometry"].get("knuckles") == "mesh" else {})
    selected = np.flatnonzero((times >= start - 1e-5) & (times <= end + 1e-5))
    rows, failure = [], None
    for k in selected:
        transforms = robot.fk(joints[k])
        contacts = support_contact_indices(transforms, rope[k], side, finger, meshes)
        candidates = np.intersect1d(contacts, arc)
        try:
            if not rows and center not in candidates:
                raise ValueError("Declared helper material is absent at phase acquisition")
            center, patch, extras = track_support_patch(candidates, len(rope[k]), center)
        except ValueError as error:
            failure = dict(time_s=float(times[k]), reason=str(error), previous_center=center)
            break
        rows.append(dict(time_s=float(times[k]), center=center, patch=patch.tolist(),
                         other_old_patches=[x.tolist() for x in extras],
                         other_material=np.setdiff1d(contacts, arc).tolist()))
    expected = np.arange(round(start * fps), round(end * fps) + 1) / fps
    complete = bool(failure is None and len(rows) == len(expected)
                    and np.allclose([r["time_s"] for r in rows], expected, atol=1e-5, rtol=0))
    report = dict(scope=__doc__, intent=intent, completed_figure=False,
                  release_completed=False, expected_interval_s=[start, end],
                  sampling_fps=fps,
                  recorded_end_s=float(times[-1]), helper_track_complete=complete,
                  failure=failure, samples=rows)
    destination = args.output or args.source / "temporary_helper_audit.json"
    destination.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k not in ("samples", "intent")}, indent=2))
    return 0 if complete else 1


if __name__ == "__main__":
    raise SystemExit(main())
