"""Check near-strand proximity and accidental old-loop pairing during motion.

This checks each recorded frame, without requiring a stationary hold. Contact
fractions and edges may change. It is not a force, topology, collision or
original-support certificate; run those separate diagnostics as well.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.pinch import near_pinch_edge_samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--start", type=float, required=True)
    args = parser.parse_args()
    with np.load(args.source / "trajectory.npz") as archive:
        times, joints, rope = (archive[key] for key in ("time", "joints", "rope"))
    if not np.isfinite(args.start) or not times[0] <= args.start < times[-1]:
        parser.error("Start must be finite and inside the recorded interval")
    selected = np.flatnonzero(times >= args.start - 1e-5)
    if abs(times[selected[0]] - args.start) > 1e-5:
        parser.error("Start must match a recorded frame")
    payload = json.loads((args.source / "trajectory.json").read_text())
    plan = json.loads((args.source / "transfer_plan.json").read_text())
    robot = RevoTron()
    checks, first_failures, rows = {}, {}, []
    for side in ("left", "right"):
        checks[side + "_near_proximity_each_frame"] = True
        checks[side + "_no_old_strand_paired_samples"] = True
    for k in selected:
        tf = robot.fk(joints[k])
        row = dict(time_s=float(times[k]), sides={})
        for side in ("left", "right"):
            paired = {}
            for strand in ("near_indices", "old_loop_indices"):
                ids = np.asarray(plan[side][strand])
                edges = ids[np.isin((ids + 1) % len(rope[k]), ids)]
                samples = near_pinch_edge_samples(tf, rope[k], side, edges,
                                                  payload["config"]["radius"])
                paired[strand] = sorted({v["edge"] for v in samples if v["paired"]})
            row["sides"][side] = paired
            for suffix, passed in (
                ("_near_proximity_each_frame", bool(paired["near_indices"])),
                ("_no_old_strand_paired_samples", not paired["old_loop_indices"]),
            ):
                key = side + suffix
                if not passed:
                    checks[key] = False
                    first_failures.setdefault(key, float(times[k]))
        rows.append(row)
    report = dict(scope=__doc__, maximum_surface_gap_m=.0001,
                  interval_s=[float(times[selected[0]]), float(times[-1])],
                  checks=checks, passed_diagnostics=all(checks.values()),
                  first_failures=first_failures, samples=rows)
    (args.source / "dynamic_near_grip_audit.json").write_text(
        json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}, indent=2))
    return 0 if report["passed_diagnostics"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
