"""Check persistent same-material thumb/index contact during a stationary hold.

This is a sampled capsule-contact diagnostic, not a force-closure or figure
certificate. Run the physical release audit and original-support tracker too.
Edge mode permits the contacted fraction to move within one persistent material
edge; it does not establish persistence of a particular point on that edge.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.pinch import near_pinch_samples, near_pinch_edge_samples
from scripts.audit_loop_release import completed_stationary_hold


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--side", choices=("left", "right", "both"), required=True)
    parser.add_argument("--sampling", choices=("points", "edges"), default="points",
                        help="Edges test 21 common fractions per near-strand edge")
    args = parser.parse_args()
    with np.load(args.source / "trajectory.npz") as archive:
        times, joints, rope = (archive[name] for name in ("time", "joints", "rope"))
    payload = json.loads((args.source / "trajectory.json").read_text())
    plan = json.loads((args.source / "transfer_plan.json").read_text())
    sides = ("left", "right") if args.side == "both" else (args.side,)
    robot = RevoTron()
    selected = np.flatnonzero(times >= times[-1] - 1 - 1e-5)
    common = {side: set(plan[side]["near_indices"]) for side in sides}
    material_key = "particle" if args.sampling == "points" else "edge"
    rows = []
    for k in selected:
        tf = robot.fk(joints[k])
        row = dict(time_s=float(times[k]), sides={})
        for side in sides:
            ids = np.array(plan[side]["near_indices"])
            if args.sampling == "edges":
                ids = ids[np.isin((ids + 1) % len(rope[k]), ids)]
                samples = near_pinch_edge_samples(tf, rope[k], side, ids,
                                                  payload["config"]["radius"])
            else:
                samples = near_pinch_samples(tf, rope[k], side, ids,
                                             payload["config"]["radius"])
            paired = [sample for sample in samples if sample["paired"]]
            common[side].intersection_update(sample[material_key] for sample in paired)
            row["sides"][side] = paired
        rows.append(row)
    checks = dict(completed_stationary_hold=completed_stationary_hold(
        times, joints, payload["motion"]["duration"]))
    checks.update({side + "_same_material_pinch": bool(common[side]) for side in sides})
    report = dict(scope=__doc__, sampling=args.sampling, checks=checks,
                  passed_diagnostics=all(checks.values()),
                  interval_s=[float(times[selected[0]]), float(times[-1])],
                  persistent_material={side: sorted(ids) for side, ids in common.items()},
                  samples=rows)
    name = "near_pinch_edge_audit.json" if args.sampling == "edges" else "near_pinch_audit.json"
    (args.source / name).write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: value for key, value in report.items() if key != "samples"},
                     indent=2))
    return 0 if report["passed_diagnostics"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
