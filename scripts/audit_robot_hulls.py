"""Detect rope centerlines entering conservative robot arm/body hulls.

Hulls fill cavities. A hit is an intrusion into the simulation proxy, not
necessarily the original mesh. No hits do not certify rope-radius clearance,
robot self-collisions, swept collisions, contact identity or figure topology.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.robot_geometry import RobotHullAudit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--inset", type=float, default=1e-7)
    args = parser.parse_args()
    if not np.isfinite(args.inset) or args.inset < 0:
        parser.error("Inset must be finite and nonnegative")
    with np.load(args.source / "trajectory.npz") as archive:
        times, joints, rope = (archive[name] for name in ("time", "joints", "rope"))
    robot = RevoTron()
    audit = RobotHullAudit(robot)
    hits = []
    for time, q, points in zip(times, joints, rope):
        result = audit.measure(robot.fk(q), points, args.inset)
        if result:
            hits.append(dict(time_s=float(time), links=result))
    report = dict(
        scope=__doc__,
        interval_s=[float(times[0]), float(times[-1])],
        inset_m=args.inset,
        frames_checked=len(times),
        frames_with_intrusions=len(hits),
        centerline_intrusions_absent=not hits,
        hits=hits,
    )
    (args.source / "robot_hull_audit.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "hits"}, indent=2))
    if hits:
        print(json.dumps({"first_intrusion": hits[0]}))
    return 1 if hits else 0


if __name__ == "__main__":
    raise SystemExit(main())
