"""Locate stretched material edges and their nearest finger capsules.

This reads a terminal checkpoint without advancing or repairing it. Multiple
nearby fingers identify a possible passage to inspect, not proof of a pinch
force or the cause of the stretch. Meshes and body surfaces are not included.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from scripts.audit_finger_contacts import audit_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--count", type=int, default=5)
    args = parser.parse_args()
    checkpoint = args.source / "checkpoint.npz"
    with np.load(checkpoint) as saved:
        points, joints, rest = (saved[k].copy() for k in
                               ("particle_q", "joints", "rest_lengths"))
        time = float(saved["time"])
        radius = float(json.loads(str(saved["config"]))["radius"])
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 3
            or rest.shape != (len(points),) or joints.ndim != 1
            or not 1 <= args.count <= len(points) or np.any(rest <= 0)
            or not all(np.isfinite(x).all() for x in (points, joints, rest))
            or not np.isfinite([time, radius]).all() or radius <= 0):
        parser.error("Checkpoint must contain finite rope state and positive rest lengths")
    robot = RevoTron()
    if len(joints) != len(robot.names):
        parser.error("Checkpoint joints do not match RevoTron")
    plan_path = args.source / "transfer_plan.json"
    plan = json.loads(plan_path.read_text()) if plan_path.exists() else {}
    stop_path = args.source / "stretch_stop.json"
    stop = json.loads(stop_path.read_text()) if stop_path.exists() else None
    if stop and abs(float(stop["time_s"]) - time) > 1e-5:
        parser.error("Stretch stop and terminal checkpoint times differ")
    lengths = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    ratios = lengths / rest
    rows = []
    for edge in np.argsort(ratios)[-args.count:][::-1]:
        edge = int(edge)
        contacts = audit_frame(robot, joints, points, radius, edge_indices=[edge])
        regions = []
        for side in ("left", "right"):
            for arc in ("far_indices", "near_indices", "old_loop_indices"):
                material = plan.get(side, {}).get(arc, [])
                if edge in material and (edge + 1) % len(points) in material:
                    regions.append(side + "_" + arc)
        rows.append(dict(edge=edge, length_m=float(lengths[edge]),
                         rest_length_m=float(rest[edge]), ratio=float(ratios[edge]),
                         material_regions=regions, nearest_capsules=contacts[:5]))
    report = dict(scope=__doc__, time_s=time, rope_radius_m=radius, stop_event=stop,
                  checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
                  edges=rows)
    output = args.source / "stretch_diagnostic.json"
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(dict(time_s=time, edges=[dict(
        edge=row["edge"], ratio=row["ratio"],
        closest_capsules=[dict(finger=c["finger"], gap_m=c["edge_gap_m"])
                          for c in row["nearest_capsules"][:2]]) for row in rows]), indent=2))


if __name__ == "__main__":
    main()
